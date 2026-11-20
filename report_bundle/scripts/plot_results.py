#!/usr/bin/env python3
"""Create all required Homework 3 plots from the ns-3 results CSV."""
import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt


def read_rows(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def select(rows, experiment, **criteria):
    out = []
    for r in rows:
        if r['experiment'] != experiment:
            continue
        ok = True
        for key, value in criteria.items():
            if key in ('protocol',):
                ok &= r[key] == value
            else:
                ok &= abs(float(r[key]) - float(value)) < 1e-9
        if ok:
            out.append(r)
    return out


def num(rows, key):
    return [float(r[key]) for r in rows]


def make_plots(rows, outdir, source_label):
    outdir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': 0.25})

    # P1(a): Stop-and-Wait goodput against round-trip propagation delay.
    r = sorted(select(rows, 'p1_delay'), key=lambda x: float(x['rtt_propagation_ms']))
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.plot(num(r, 'rtt_propagation_ms'), num(r, 'goodput_mbps'), marker='o', linewidth=2)
    ax.set_xlabel('Round-trip propagation delay (ms)')
    ax.set_ylabel('Goodput (Mbps)')
    ax.set_title('Stop-and-Wait: Goodput vs. propagation RTT')
    ax.text(0.99, 0.02, source_label, transform=ax.transAxes, ha='right', va='bottom', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p1a_stop_wait_delay.png', dpi=300); plt.close(fig)

    # P1(b): Goodput and retransmission ratio against frame-loss probability.
    r = sorted(select(rows, 'p1_loss'), key=lambda x: float(x['loss_probability']))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    axes[0].plot(num(r, 'loss_probability'), num(r, 'goodput_mbps'), marker='o', linewidth=2)
    axes[0].set_xlabel('Independent DATA / ACK frame-loss probability, p')
    axes[0].set_ylabel('Goodput (Mbps)')
    axes[0].set_title('Goodput')
    axes[1].plot(num(r, 'loss_probability'), num(r, 'retransmissions_per_unique'), marker='s', linewidth=2)
    axes[1].set_xlabel('Independent DATA / ACK frame-loss probability, p')
    axes[1].set_ylabel('Retransmissions per unique delivered DATA')
    axes[1].set_title('Retransmission ratio')
    fig.suptitle('Stop-and-Wait: Effect of frame loss')
    fig.text(0.99, 0.01, source_label, ha='right', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p1b_stop_wait_loss.png', dpi=300); plt.close(fig)

    # P2(a): GBN goodput versus window for two propagation delays.
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    for d, label in [(5.0, '5 ms one-way'), (20.0, '20 ms one-way')]:
        r = sorted(select(rows, 'p2_gbn_bdp', delay_ms=d), key=lambda x: int(x['window']))
        ax.plot(num(r, 'window'), num(r, 'goodput_mbps'), marker='o', linewidth=2, label=label)
    ax.set_xlabel('Sender window size, W (packets)')
    ax.set_ylabel('Goodput (Mbps)')
    ax.set_title('Go-Back-N: Goodput vs. sender window')
    ax.set_xticks([1, 2, 4, 8, 16, 32])
    ax.legend()
    ax.text(0.99, 0.02, source_label, transform=ax.transAxes, ha='right', va='bottom', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p2a_gbn_window_goodput.png', dpi=300); plt.close(fig)

    # P3(a): GBN and SR under the same p=0.05 conditions.
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    for proto, label, marker in [('gbn', 'Go-Back-N', 'o'), ('sr', 'Selective Repeat', 's')]:
        r = sorted(select(rows, 'p3_compare', protocol=proto), key=lambda x: int(x['window']))
        ax.plot(num(r, 'window'), num(r, 'goodput_mbps'), marker=marker, linewidth=2, label=label)
    ax.set_xlabel('Sender / receiver window size, W (packets)')
    ax.set_ylabel('Goodput (Mbps)')
    ax.set_title('Go-Back-N vs. Selective Repeat (p = 0.05)')
    ax.set_xticks([1, 2, 4, 8, 16, 32])
    ax.legend()
    ax.text(0.99, 0.02, source_label, transform=ax.transAxes, ha='right', va='bottom', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p3a_gbn_vs_sr_goodput.png', dpi=300); plt.close(fig)

    # P3(b): SR receiver capacity, goodput and peak occupancy.
    r = sorted(select(rows, 'p3_buffer'), key=lambda x: int(x['buffer_capacity']))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    axes[0].plot(num(r, 'buffer_capacity'), num(r, 'goodput_mbps'), marker='o', linewidth=2)
    axes[0].set_xlabel('Receiver out-of-order buffer capacity (packets)')
    axes[0].set_ylabel('Goodput (Mbps)')
    axes[0].set_title('Goodput')
    axes[1].plot(num(r, 'buffer_capacity'), num(r, 'peak_buffer_occupancy'), marker='s', linewidth=2, label='Measured peak')
    axes[1].plot(num(r, 'buffer_capacity'), num(r, 'buffer_capacity'), linestyle='--', label='Capacity limit')
    axes[1].set_xlabel('Receiver buffer capacity (packets)')
    axes[1].set_ylabel('Peak buffer occupancy (packets)')
    axes[1].set_title('Buffer occupancy')
    axes[1].legend()
    fig.suptitle('Selective Repeat: Effect of receiver buffering (W = 32, p = 0.05)')
    fig.text(0.99, 0.01, source_label, ha='right', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p3b_sr_buffer_capacity.png', dpi=300); plt.close(fig)

    # Extra diagnostic plot: mean delivery delay and retransmissions for P3(a).
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    for proto, label in [('gbn', 'Go-Back-N'), ('sr', 'Selective Repeat')]:
        r = sorted(select(rows, 'p3_compare', protocol=proto), key=lambda x: int(x['window']))
        axes[0].plot(num(r, 'window'), num(r, 'mean_delivery_delay_ms'), marker='o', label=label)
        axes[1].plot(num(r, 'window'), num(r, 'retransmissions_per_unique'), marker='s', label=label)
    axes[0].set_xlabel('Window size, W (packets)'); axes[0].set_ylabel('Mean delivery delay (ms)')
    axes[0].set_title('Mean delivery delay')
    axes[1].set_xlabel('Window size, W (packets)'); axes[1].set_ylabel('Retransmissions per unique DATA')
    axes[1].set_title('Retransmission ratio')
    for ax in axes: ax.legend()
    fig.suptitle('P3 diagnostics: Go-Back-N vs. Selective Repeat')
    fig.text(0.99, 0.01, source_label, ha='right', fontsize=8)
    fig.tight_layout(); fig.savefig(outdir / 'p3c_delay_retransmission.png', dpi=300); plt.close(fig)

    print(f'Created plots in {outdir}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_file')
    ap.add_argument('--outdir', default='plots')
    ap.add_argument('--source-label', default='Source: CSV input')
    args = ap.parse_args()
    rows = read_rows(args.csv_file)
    make_plots(rows, Path(args.outdir), args.source_label)

if __name__ == '__main__':
    main()
