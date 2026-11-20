#!/usr/bin/env python3
"""Deterministic event-driven reference model for the ns-3 ARQ assignment.

This script mirrors the protocol state machines in scratch/arq-hw3.cc and is
provided so the report/plots can be previewed when the supplied Docker/ns-3
runtime is not available. It is NOT an ns-3 execution. Run the C++ scratch
program in the course container to obtain the authoritative ns-3 CSV.
"""
from __future__ import annotations
import argparse
import csv
import heapq
import math
import random
from dataclasses import dataclass, asdict
from pathlib import Path

@dataclass
class Config:
    experiment: str
    protocol: str
    delay_ms: float = 5.0
    loss_probability: float = 0.0
    window: int = 1
    buffer_capacity: int = 20
    payload_bytes: int = 1000
    rate_mbps: float = 10.0
    simulation_s: float = 30.0
    timeout_ms: float = 100.0
    seed: int = 12345

class EventSimulator:
    def __init__(self, c: Config):
        self.c = c
        self.now = 0.0
        self.q = []
        self.counter = 0
        self.data_available = 0.0
        self.ack_available = 0.0
        self.data_rng = random.Random(c.seed + 10)
        self.ack_rng = random.Random(c.seed + 11)
        self.data_wire_bytes = c.payload_bytes + 5 + 2  # custom header + PPP header
        self.ack_wire_bytes = 5 + 2
        self.data_tx = self.data_wire_bytes * 8 / (c.rate_mbps * 1e6)
        self.ack_tx = self.ack_wire_bytes * 8 / (c.rate_mbps * 1e6)
        self.delay = c.delay_ms / 1000.0
        self.timeout = c.timeout_ms / 1000.0
        self.base = 0
        self.next_seq = 0
        self.rx_next = 0
        self.tx = {}
        self.first_tx = {}
        self.rx_buffer = {}
        self.delivered = set()
        self.delay_sum = 0.0
        self.retransmissions = 0
        self.timeout_count = 0
        self.data_drops = 0
        self.ack_drops = 0
        self.buffer_overflow = 0
        self.peak_buffer = 0
        self.sw_token = 0
        self.gbn_token = 0
        self.sr_tokens = {}

    def schedule(self, t, kind, seq=0, token=0):
        self.counter += 1
        heapq.heappush(self.q, (t, self.counter, kind, seq, token))

    def link_send(self, direction, frame_kind, seq):
        if direction == 'data':
            start = max(self.now, self.data_available)
            arrival = start + self.data_tx + self.delay
            self.data_available = start + self.data_tx
        else:
            start = max(self.now, self.ack_available)
            arrival = start + self.ack_tx + self.delay
            self.ack_available = start + self.ack_tx
        self.schedule(arrival, frame_kind, seq)

    def transmit(self, seq):
        state = self.tx.setdefault(seq, {'attempts': 0, 'acked': False})
        state['attempts'] += 1
        if state['attempts'] == 1:
            self.first_tx[seq] = self.now
        else:
            self.retransmissions += 1
        self.link_send('data', 'data_arrival', seq)
        if self.c.protocol == 'sw':
            self.sw_token += 1
            self.schedule(self.now + self.timeout, 'sw_timeout', seq, self.sw_token)
        elif self.c.protocol == 'sr':
            token = self.sr_tokens.get(seq, 0) + 1
            self.sr_tokens[seq] = token
            self.schedule(self.now + self.timeout, 'sr_timeout', seq, token)

    def start(self):
        self.fill_window()

    def fill_window(self):
        if self.c.protocol == 'sw':
            if self.base == self.next_seq:
                seq = self.next_seq
                self.next_seq += 1
                self.transmit(seq)
            return
        if self.c.protocol == 'gbn':
            was_empty = self.base == self.next_seq
            while self.next_seq < self.base + self.c.window:
                seq = self.next_seq
                self.next_seq += 1
                self.transmit(seq)
            if was_empty and self.base < self.next_seq:
                self.start_gbn_timer()
            return
        while self.next_seq < self.base + self.c.window:
            seq = self.next_seq
            self.next_seq += 1
            self.transmit(seq)

    def start_gbn_timer(self):
        self.gbn_token += 1
        self.schedule(self.now + self.timeout, 'gbn_timeout', self.base, self.gbn_token)

    def send_ack(self, seq):
        self.link_send('ack', 'ack_arrival', seq)

    def deliver(self, seq):
        if seq in self.delivered:
            return
        self.delivered.add(seq)
        self.delay_sum += self.now - self.first_tx.get(seq, self.now)

    def on_data(self, seq):
        if self.data_rng.random() < self.c.loss_probability:
            self.data_drops += 1
            return
        if self.c.protocol == 'sw':
            if seq == self.rx_next:
                self.deliver(seq)
                self.rx_next += 1
            self.send_ack(seq)  # individual ACK, including duplicate re-ACK
            return
        if self.c.protocol == 'gbn':
            if seq == self.rx_next:
                self.deliver(seq)
                self.rx_next += 1
            self.send_ack(self.rx_next)  # cumulative ACK = next expected
            return
        # Selective Repeat receiver
        if seq < self.rx_next:
            self.send_ack(seq)
            return
        if seq >= self.rx_next + self.c.window:
            return
        if seq == self.rx_next:
            self.deliver(seq)
            self.rx_next += 1
            while self.rx_next in self.rx_buffer:
                del self.rx_buffer[self.rx_next]
                self.deliver(self.rx_next)
                self.rx_next += 1
            self.send_ack(seq)
            return
        if seq in self.rx_buffer:
            self.send_ack(seq)
            return
        if len(self.rx_buffer) >= self.c.buffer_capacity:
            self.buffer_overflow += 1
            return
        self.rx_buffer[seq] = True
        self.peak_buffer = max(self.peak_buffer, len(self.rx_buffer))
        self.send_ack(seq)

    def on_ack(self, seq):
        if self.ack_rng.random() < self.c.loss_probability:
            self.ack_drops += 1
            return
        if self.c.protocol == 'sw':
            if seq == self.base and seq in self.tx:
                self.sw_token += 1  # invalidate old timer
                self.tx.pop(seq, None)
                self.base += 1
                self.fill_window()
            return
        if self.c.protocol == 'gbn':
            if seq <= self.base or seq > self.next_seq:
                return
            for s in range(self.base, seq):
                self.tx.pop(s, None)
            self.base = seq
            self.gbn_token += 1  # invalidate old timer
            if self.base < self.next_seq:
                self.start_gbn_timer()
            self.fill_window()
            return
        state = self.tx.get(seq)
        if state is None or seq < self.base or seq >= self.next_seq:
            return
        state['acked'] = True
        self.sr_tokens[seq] = self.sr_tokens.get(seq, 0) + 1
        while self.base in self.tx and self.tx[self.base]['acked']:
            self.tx.pop(self.base, None)
            self.base += 1
        self.fill_window()

    def on_timeout(self, kind, seq, token):
        if kind == 'sw_timeout':
            if token != self.sw_token or seq != self.base or seq not in self.tx:
                return
            self.timeout_count += 1
            self.transmit(seq)
        elif kind == 'gbn_timeout':
            if token != self.gbn_token or self.base >= self.next_seq:
                return
            self.timeout_count += 1
            for s in range(self.base, self.next_seq):
                if s in self.tx:
                    state = self.tx[s]
                    state['attempts'] += 1
                    self.retransmissions += 1
                    self.link_send('data', 'data_arrival', s)
            self.start_gbn_timer()
        else:
            if token != self.sr_tokens.get(seq) or seq not in self.tx or self.tx[seq]['acked']:
                return
            self.timeout_count += 1
            self.transmit(seq)

    def run(self):
        self.start()
        while self.q:
            t, _, kind, seq, token = heapq.heappop(self.q)
            if t > self.c.simulation_s:
                break
            self.now = t
            if kind == 'data_arrival':
                self.on_data(seq)
            elif kind == 'ack_arrival':
                self.on_ack(seq)
            else:
                self.on_timeout(kind, seq, token)
        delivered = len(self.delivered)
        goodput = delivered * self.c.payload_bytes * 8 / self.c.simulation_s / 1e6
        mean_delay_ms = (self.delay_sum / delivered * 1000) if delivered else 0.0
        return {
            'experiment': self.c.experiment,
            'protocol': self.c.protocol,
            'delay_ms': self.c.delay_ms,
            'rtt_propagation_ms': 2 * self.c.delay_ms,
            'loss_probability': self.c.loss_probability,
            'window': self.c.window,
            'buffer_capacity': self.c.buffer_capacity,
            'payload_bytes': self.c.payload_bytes,
            'rate_mbps': self.c.rate_mbps,
            'simulation_s': self.c.simulation_s,
            'timeout_ms': self.c.timeout_ms,
            'seed': self.c.seed,
            'unique_delivered': delivered,
            'goodput_mbps': goodput,
            'mean_delivery_delay_ms': mean_delay_ms,
            'retransmissions': self.retransmissions,
            'retransmissions_per_unique': self.retransmissions / delivered if delivered else 0.0,
            'timeout_count': self.timeout_count,
            'data_drops': self.data_drops,
            'ack_drops': self.ack_drops,
            'buffer_overflow': self.buffer_overflow,
            'peak_buffer_occupancy': self.peak_buffer,
        }

def run_one(c):
    return EventSimulator(c).run()

def all_configs(args):
    common = dict(payload_bytes=args.payload_bytes, rate_mbps=args.rate_mbps,
                  simulation_s=args.simulation_s, timeout_ms=args.timeout_ms, seed=args.seed)
    configs = []
    for d in (1.0, 5.0, 20.0):
        configs.append(Config('p1_delay', 'sw', delay_ms=d, loss_probability=0,
                              window=1, buffer_capacity=20, **common))
    for p in (0.0, 0.01, 0.05, 0.10, 0.20):
        configs.append(Config('p1_loss', 'sw', delay_ms=5, loss_probability=p,
                              window=1, buffer_capacity=20, **common))
    for d in (5.0, 20.0):
        for w in (1, 2, 4, 8, 16, 32):
            configs.append(Config('p2_gbn_bdp', 'gbn', delay_ms=d, loss_probability=0,
                                  window=w, buffer_capacity=20, **common))
    for proto in ('gbn', 'sr'):
        for w in (1, 2, 4, 8, 16, 32):
            configs.append(Config('p3_compare', proto, delay_ms=5, loss_probability=0.05,
                                  window=w, buffer_capacity=20, **common))
    for b in (4, 8, 16, 32, 64):
        configs.append(Config('p3_buffer', 'sr', delay_ms=5, loss_probability=0.05,
                              window=32, buffer_capacity=b, **common))
    return configs

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='results/reference_results.csv')
    ap.add_argument('--payload-bytes', type=int, default=1000)
    ap.add_argument('--rate-mbps', type=float, default=10)
    ap.add_argument('--simulation-s', type=float, default=30)
    ap.add_argument('--timeout-ms', type=float, default=100)
    ap.add_argument('--seed', type=int, default=12345)
    args = ap.parse_args()
    rows = []
    for i, c in enumerate(all_configs(args), 1):
        row = run_one(c)
        rows.append(row)
        print(f"[{i:02d}/{len(all_configs(args))}] {c.experiment:12s} {c.protocol:3s} "
              f"d={c.delay_ms:4.0f}ms p={c.loss_probability:.2f} W={c.window:2d} B={c.buffer_capacity:2d} "
              f"goodput={row['goodput_mbps']:.3f} Mbps retx={row['retransmissions']} "
              f"timeouts={row['timeout_count']}")
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} reference rows to {path}")

if __name__ == '__main__':
    main()
