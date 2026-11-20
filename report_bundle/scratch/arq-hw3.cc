/*
 * ns-3 Homework 3: Flow Control and Reliable Transmission
 *
 * A custom DATA/ACK ARQ entity implemented directly above two PointToPointNetDevices.
 * No Wi-Fi, IP, UDP, or TCP is used. Exactly two ns-3 nodes are connected by one
 * point-to-point channel. The ns-3 PointToPointNetDevice only provides framing,
 * serialization, and propagation; all ARQ behavior is implemented here.
 *
 * Compatible target: ns-3.46 (as referenced by the supplied Docker/build guide).
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

namespace {

constexpr uint16_t kArqProtocol = 0x0800;
constexpr uint8_t kDataFrame = 1;
constexpr uint8_t kAckFrame = 2;

class ArqHeader : public Header
{
public:
    ArqHeader() = default;
    ArqHeader(uint8_t type, uint32_t sequence) : m_type(type), m_sequence(sequence) {}

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::ArqHeader")
                                .SetParent<Header>()
                                .AddConstructor<ArqHeader>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 5; }

    void Serialize(Buffer::Iterator start) const override
    {
        start.WriteU8(m_type);
        start.WriteHtonU32(m_sequence);
    }

    uint32_t Deserialize(Buffer::Iterator start) override
    {
        m_type = start.ReadU8();
        m_sequence = start.ReadNtohU32();
        return GetSerializedSize();
    }

    void Print(std::ostream& os) const override
    {
        os << "type=" << unsigned(m_type) << " seq=" << m_sequence;
    }

    uint8_t GetType() const { return m_type; }
    uint32_t GetSequence() const { return m_sequence; }

private:
    uint8_t m_type{0};
    uint32_t m_sequence{0};
};

struct ArqConfig
{
    std::string experiment{"single"};
    std::string protocol{"sr"};
    double rateMbps{10.0};
    double delayMs{5.0};
    uint32_t payloadBytes{1000};
    double simulationSeconds{30.0};
    double timeoutMs{100.0};
    double lossProbability{0.0};
    uint32_t window{4};
    uint32_t receiverBuffer{20};
    uint32_t seed{12345};
};

struct Result
{
    ArqConfig config;
    uint64_t uniqueDelivered{0};
    double goodputMbps{0.0};
    double meanDelayMs{0.0};
    uint64_t retransmissions{0};
    double retransmissionsPerUnique{0.0};
    uint64_t timeoutCount{0};
    uint64_t dataDrops{0};
    uint64_t ackDrops{0};
    uint64_t bufferOverflow{0};
    uint32_t peakBuffer{0};
};

class ArqExperiment
{
public:
    explicit ArqExperiment(const ArqConfig& config) : m_config(config)
    {
        m_mode = ParseMode(config.protocol);

        NodeContainer nodes;
        nodes.Create(2);

        PointToPointHelper pointToPoint;
        pointToPoint.SetDeviceAttribute("DataRate",
                                         StringValue(std::to_string(config.rateMbps) + "Mbps"));
        pointToPoint.SetChannelAttribute("Delay", TimeValue(MilliSeconds(config.delayMs)));
        NetDeviceContainer devices = pointToPoint.Install(nodes);

        m_senderNode = nodes.Get(0);
        m_receiverNode = nodes.Get(1);
        m_senderDevice = devices.Get(0);
        m_receiverDevice = devices.Get(1);

        m_dataLoss = CreateObject<UniformRandomVariable>();
        m_ackLoss = CreateObject<UniformRandomVariable>();
        m_dataLoss->SetStream(10);
        m_ackLoss->SetStream(11);

        // DATA is accepted only at node 1; ACK is accepted only at node 0.
        m_receiverNode->RegisterProtocolHandler(
            MakeCallback(&ArqExperiment::ReceiveData, this), kArqProtocol, m_receiverDevice);
        m_senderNode->RegisterProtocolHandler(
            MakeCallback(&ArqExperiment::ReceiveAck, this), kArqProtocol, m_senderDevice);
    }

    void Run()
    {
        Simulator::Schedule(Seconds(0), &ArqExperiment::Start, this);
        Simulator::Stop(Seconds(m_config.simulationSeconds));
        Simulator::Run();
    }

    Result GetResult() const
    {
        Result result;
        result.config = m_config;
        result.uniqueDelivered = m_delivered.size();
        result.goodputMbps = (m_delivered.size() * m_config.payloadBytes * 8.0) /
                             m_config.simulationSeconds / 1e6;
        result.meanDelayMs = m_delivered.empty()
                                 ? 0.0
                                 : (m_delaySumSeconds / m_delivered.size()) * 1000.0;
        result.retransmissions = m_retransmissions;
        result.retransmissionsPerUnique = m_delivered.empty()
                                              ? 0.0
                                              : double(m_retransmissions) / m_delivered.size();
        result.timeoutCount = m_timeoutCount;
        result.dataDrops = m_dataDrops;
        result.ackDrops = m_ackDrops;
        result.bufferOverflow = m_bufferOverflow;
        result.peakBuffer = m_peakBuffer;
        return result;
    }

private:
    enum Mode { STOP_WAIT, GO_BACK_N, SELECTIVE_REPEAT };

    struct TxState
    {
        uint32_t attempts{0};
        bool acked{false};
        EventId timer;
    };

    static Mode ParseMode(const std::string& protocol)
    {
        if (protocol == "sw" || protocol == "stop-wait") return STOP_WAIT;
        if (protocol == "gbn" || protocol == "go-back-n") return GO_BACK_N;
        if (protocol == "sr" || protocol == "selective-repeat") return SELECTIVE_REPEAT;
        NS_FATAL_ERROR("Unknown protocol: " << protocol << ". Use sw, gbn, or sr.");
        return SELECTIVE_REPEAT;
    }

    Time Timeout() const { return MilliSeconds(m_config.timeoutMs); }

    void Start() { FillWindow(); }

    void FillWindow()
    {
        if (m_mode == STOP_WAIT)
        {
            if (m_base == m_nextSequence)
            {
                const uint32_t sequence = m_nextSequence++;
                TransmitData(sequence);
            }
            return;
        }

        while (m_nextSequence < m_base + m_config.window)
        {
            const uint32_t sequence = m_nextSequence++;
            TransmitData(sequence);
        }
        if (m_mode == GO_BACK_N && m_base < m_nextSequence)
        {
            StartGoBackNTimer();
        }
    }

    void StartGoBackNTimer()
    {
        if (m_gbnTimerActive || m_base >= m_nextSequence) return;
        m_gbnTimerActive = true;
        m_gbnTimer = Simulator::Schedule(Timeout(),
                                         &ArqExperiment::GoBackNTimeout,
                                         this);
    }

    void TransmitData(uint32_t sequence)
    {
        TxState& state = m_txState[sequence];
        ++state.attempts;
        if (state.attempts == 1)
        {
            m_firstTransmission[sequence] = Simulator::Now();
        }
        else
        {
            ++m_retransmissions;
        }

        Ptr<Packet> packet = Create<Packet>(m_config.payloadBytes);
        packet->AddHeader(ArqHeader(kDataFrame, sequence));
        const bool accepted = m_senderDevice->Send(packet, m_receiverDevice->GetAddress(), kArqProtocol);

        if (m_mode == STOP_WAIT)
        {
            if (m_stopWaitTimer.IsPending()) m_stopWaitTimer.Cancel();
            m_stopWaitTimer = Simulator::Schedule(Timeout(),
                                                   &ArqExperiment::StopWaitTimeout,
                                                   this,
                                                   sequence);
        }
        // Go-Back-N owns one timer for the oldest outstanding sequence. The
        // window-management code starts/restarts it; individual sends do not.
        else if (m_mode == SELECTIVE_REPEAT)
        {
            if (state.timer.IsPending()) state.timer.Cancel();
            state.timer = Simulator::Schedule(Timeout(),
                                              &ArqExperiment::SelectiveRepeatTimeout,
                                              this,
                                              sequence);
        }
    }

    void SendAck(uint32_t sequence)
    {
        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(ArqHeader(kAckFrame, sequence));
        m_receiverDevice->Send(packet, m_senderDevice->GetAddress(), kArqProtocol);
    }

    void ReceiveData(Ptr<NetDevice>,
                     Ptr<const Packet> packet,
                     uint16_t,
                     const Address&,
                     const Address&,
                     NetDevice::PacketType)
    {
        if (m_dataLoss->GetValue(0.0, 1.0) < m_config.lossProbability)
        {
            ++m_dataDrops;
            return;
        }

        Ptr<Packet> copy = packet->Copy();
        ArqHeader header;
        copy->RemoveHeader(header);
        if (header.GetType() != kDataFrame) return;
        const uint32_t sequence = header.GetSequence();

        if (m_mode == STOP_WAIT)
        {
            if (sequence == m_receiveNext)
            {
                Deliver(sequence);
                ++m_receiveNext;
                SendAck(sequence);
            }
            else if (sequence < m_receiveNext)
            {
                SendAck(sequence); // re-ACK a duplicate already delivered
            }
            return;
        }

        if (m_mode == GO_BACK_N)
        {
            if (sequence == m_receiveNext)
            {
                Deliver(sequence);
                ++m_receiveNext;
            }
            // Cumulative ACK is the next sequence expected; out-of-order frames are discarded.
            SendAck(m_receiveNext);
            return;
        }

        // Selective Repeat: accept only within the receive window. In-order data is
        // delivered immediately; out-of-order data is held in the finite RX buffer.
        if (sequence < m_receiveNext)
        {
            SendAck(sequence); // duplicate of an already delivered packet
            return;
        }

        if (sequence >= m_receiveNext + m_config.window)
        {
            return; // outside receiver window: no ACK, sender will retry
        }

        if (sequence == m_receiveNext)
        {
            Deliver(sequence);
            ++m_receiveNext;
            while (true)
            {
                auto it = m_receiveBuffer.find(m_receiveNext);
                if (it == m_receiveBuffer.end()) break;
                m_receiveBuffer.erase(it);
                Deliver(m_receiveNext);
                ++m_receiveNext;
            }
            SendAck(sequence);
            return;
        }

        auto existing = m_receiveBuffer.find(sequence);
        if (existing != m_receiveBuffer.end())
        {
            SendAck(sequence); // duplicate still waiting in the out-of-order buffer
            return;
        }

        if (m_receiveBuffer.size() >= m_config.receiverBuffer)
        {
            ++m_bufferOverflow;
            return; // overflow policy: drop without ACK; sender's per-packet timer retries
        }

        m_receiveBuffer.emplace(sequence, true);
        m_peakBuffer = std::max<uint32_t>(m_peakBuffer, m_receiveBuffer.size());
        SendAck(sequence);
    }

    void ReceiveAck(Ptr<NetDevice>,
                    Ptr<const Packet> packet,
                    uint16_t,
                    const Address&,
                    const Address&,
                    NetDevice::PacketType)
    {
        if (m_ackLoss->GetValue(0.0, 1.0) < m_config.lossProbability)
        {
            ++m_ackDrops;
            return;
        }

        Ptr<Packet> copy = packet->Copy();
        ArqHeader header;
        copy->RemoveHeader(header);
        if (header.GetType() != kAckFrame) return;
        const uint32_t ackSequence = header.GetSequence();

        if (m_mode == STOP_WAIT)
        {
            if (ackSequence == m_base && m_txState.find(m_base) != m_txState.end())
            {
                if (m_stopWaitTimer.IsPending()) m_stopWaitTimer.Cancel();
                m_txState.erase(m_base);
                ++m_base;
                FillWindow();
            }
            return;
        }

        if (m_mode == GO_BACK_N)
        {
            if (ackSequence <= m_base || ackSequence > m_nextSequence) return;
            for (uint32_t sequence = m_base; sequence < ackSequence; ++sequence)
            {
                m_txState.erase(sequence);
            }
            m_base = ackSequence;
            if (m_gbnTimerActive && m_gbnTimer.IsPending()) m_gbnTimer.Cancel();
            m_gbnTimerActive = false;
            FillWindow();
            return;
        }

        auto it = m_txState.find(ackSequence);
        if (it == m_txState.end() || ackSequence < m_base || ackSequence >= m_nextSequence)
        {
            return;
        }
        it->second.acked = true;
        if (it->second.timer.IsPending()) it->second.timer.Cancel();

        while (true)
        {
            auto baseIt = m_txState.find(m_base);
            if (baseIt == m_txState.end() || !baseIt->second.acked) break;
            if (baseIt->second.timer.IsPending()) baseIt->second.timer.Cancel();
            m_txState.erase(baseIt);
            ++m_base;
        }
        FillWindow();
    }

    void Deliver(uint32_t sequence)
    {
        if (!m_delivered.insert(sequence).second) return;
        auto it = m_firstTransmission.find(sequence);
        if (it != m_firstTransmission.end())
        {
            m_delaySumSeconds += (Simulator::Now() - it->second).GetSeconds();
        }
    }

    void StopWaitTimeout(uint32_t sequence)
    {
        if (sequence != m_base || m_txState.find(sequence) == m_txState.end()) return;
        ++m_timeoutCount;
        TransmitData(sequence);
    }

    void GoBackNTimeout()
    {
        m_gbnTimerActive = false;
        if (m_base >= m_nextSequence) return;
        ++m_timeoutCount;
        for (uint32_t sequence = m_base; sequence < m_nextSequence; ++sequence)
        {
            if (m_txState.find(sequence) != m_txState.end()) TransmitData(sequence);
        }
        StartGoBackNTimer();
    }

    void SelectiveRepeatTimeout(uint32_t sequence)
    {
        auto it = m_txState.find(sequence);
        if (it == m_txState.end() || it->second.acked) return;
        ++m_timeoutCount;
        TransmitData(sequence);
    }

    ArqConfig m_config;
    Mode m_mode{SELECTIVE_REPEAT};
    Ptr<Node> m_senderNode;
    Ptr<Node> m_receiverNode;
    Ptr<NetDevice> m_senderDevice;
    Ptr<NetDevice> m_receiverDevice;
    Ptr<UniformRandomVariable> m_dataLoss;
    Ptr<UniformRandomVariable> m_ackLoss;

    uint32_t m_base{0};
    uint32_t m_nextSequence{0};
    uint32_t m_receiveNext{0};
    std::map<uint32_t, TxState> m_txState;
    std::map<uint32_t, Time> m_firstTransmission;
    std::map<uint32_t, bool> m_receiveBuffer;
    std::set<uint32_t> m_delivered;

    EventId m_stopWaitTimer;
    EventId m_gbnTimer;
    bool m_gbnTimerActive{false};

    uint64_t m_retransmissions{0};
    uint64_t m_timeoutCount{0};
    uint64_t m_dataDrops{0};
    uint64_t m_ackDrops{0};
    uint64_t m_bufferOverflow{0};
    uint32_t m_peakBuffer{0};
    double m_delaySumSeconds{0.0};
};

Result RunOne(const ArqConfig& config)
{
    RngSeedManager::SetSeed(config.seed);
    RngSeedManager::SetRun(1);
    Result result;
    {
        ArqExperiment experiment(config);
        experiment.Run();
        result = experiment.GetResult();
        Simulator::Destroy();
    }
    return result;
}

void WriteHeader(std::ofstream& out)
{
    out << "experiment,protocol,delay_ms,rtt_propagation_ms,loss_probability,window,"
           "buffer_capacity,payload_bytes,rate_mbps,simulation_s,timeout_ms,seed,"
           "unique_delivered,goodput_mbps,mean_delivery_delay_ms,retransmissions,"
           "retransmissions_per_unique,timeout_count,data_drops,ack_drops,"
           "buffer_overflow,peak_buffer_occupancy\n";
}

void WriteResult(std::ofstream& out, const Result& r)
{
    const ArqConfig& c = r.config;
    out << c.experiment << ',' << c.protocol << ',' << std::fixed << std::setprecision(6)
        << c.delayMs << ',' << 2.0 * c.delayMs << ',' << c.lossProbability << ','
        << c.window << ',' << c.receiverBuffer << ',' << c.payloadBytes << ','
        << c.rateMbps << ',' << c.simulationSeconds << ',' << c.timeoutMs << ','
        << c.seed << ',' << r.uniqueDelivered << ',' << r.goodputMbps << ','
        << r.meanDelayMs << ',' << r.retransmissions << ','
        << r.retransmissionsPerUnique << ',' << r.timeoutCount << ','
        << r.dataDrops << ',' << r.ackDrops << ',' << r.bufferOverflow << ','
        << r.peakBuffer << '\n';
}

void RunAndWrite(std::ofstream& out, ArqConfig config)
{
    Result result = RunOne(config);
    WriteResult(out, result);
    std::cout << std::left << std::setw(18) << config.experiment
              << std::setw(8) << config.protocol
              << " d=" << std::setw(5) << config.delayMs
              << " p=" << std::setw(5) << config.lossProbability
              << " W=" << std::setw(3) << config.window
              << " B=" << std::setw(3) << config.receiverBuffer
              << " goodput=" << std::fixed << std::setprecision(3)
              << result.goodputMbps << " Mbps, delivered=" << result.uniqueDelivered
              << ", retx=" << result.retransmissions
              << ", timeouts=" << result.timeoutCount << '\n';
}

} // namespace

int main(int argc, char* argv[])
{
    std::string mode = "all";
    std::string protocol = "sr";
    std::string outFile = "results/ns3_results.csv";
    double rateMbps = 10.0;
    double delayMs = 5.0;
    double simulationSeconds = 30.0;
    double timeoutMs = 100.0;
    double loss = 0.0;
    uint32_t payloadBytes = 1000;
    uint32_t window = 4;
    uint32_t bufferCapacity = 20;
    uint32_t seed = 12345;

    CommandLine cmd(__FILE__);
    cmd.AddValue("mode", "all, or one of: single, stop-delay, stop-loss, gbn-bdp, sr-compare, sr-buffer", mode);
    cmd.AddValue("protocol", "Protocol for single mode: sw, gbn, or sr", protocol);
    cmd.AddValue("outFile", "CSV output path (parent directory must exist)", outFile);
    cmd.AddValue("rateMbps", "Point-to-point data rate in Mbps", rateMbps);
    cmd.AddValue("delayMs", "One-way propagation delay in milliseconds", delayMs);
    cmd.AddValue("simulationSeconds", "Simulation duration in seconds", simulationSeconds);
    cmd.AddValue("timeoutMs", "ARQ timeout in milliseconds", timeoutMs);
    cmd.AddValue("loss", "Independent DATA and ACK frame-loss probability", loss);
    cmd.AddValue("payloadBytes", "Unique DATA payload size in bytes", payloadBytes);
    cmd.AddValue("window", "Sender/receiver window size", window);
    cmd.AddValue("bufferCapacity", "Selective Repeat out-of-order buffer capacity", bufferCapacity);
    cmd.AddValue("seed", "Fixed ns-3 random seed", seed);
    cmd.Parse(argc, argv);

    std::ofstream out(outFile.c_str(), std::ios::out | std::ios::trunc);
    if (!out)
    {
        std::cerr << "Cannot open output CSV: " << outFile
                  << " (create its parent directory first).\n";
        return 2;
    }
    WriteHeader(out);

    ArqConfig base;
    base.rateMbps = rateMbps;
    base.payloadBytes = payloadBytes;
    base.simulationSeconds = simulationSeconds;
    base.timeoutMs = timeoutMs;
    base.seed = seed;

    if (mode == "single")
    {
        base.experiment = "single";
        base.protocol = protocol;
        base.delayMs = delayMs;
        base.lossProbability = loss;
        base.window = window;
        base.receiverBuffer = bufferCapacity;
        RunAndWrite(out, base);
        out.close();
        return 0;
    }
    else if (mode == "all" || mode == "stop-delay")
    {
        for (double d : {1.0, 5.0, 20.0})
        {
            ArqConfig c = base;
            c.experiment = "p1_delay";
            c.protocol = "sw";
            c.delayMs = d;
            c.lossProbability = 0.0;
            c.window = 1;
            RunAndWrite(out, c);
        }
        if (mode == "stop-delay") return 0;
    }

    if (mode == "all" || mode == "stop-loss")
    {
        for (double p : {0.0, 0.01, 0.05, 0.10, 0.20})
        {
            ArqConfig c = base;
            c.experiment = "p1_loss";
            c.protocol = "sw";
            c.delayMs = 5.0;
            c.lossProbability = p;
            c.window = 1;
            RunAndWrite(out, c);
        }
        if (mode == "stop-loss") return 0;
    }

    if (mode == "all" || mode == "gbn-bdp")
    {
        for (double d : {5.0, 20.0})
        {
            for (uint32_t w : {1u, 2u, 4u, 8u, 16u, 32u})
            {
                ArqConfig c = base;
                c.experiment = "p2_gbn_bdp";
                c.protocol = "gbn";
                c.delayMs = d;
                c.lossProbability = 0.0;
                c.window = w;
                c.receiverBuffer = 20;
                RunAndWrite(out, c);
            }
        }
        if (mode == "gbn-bdp") return 0;
    }

    if (mode == "all" || mode == "sr-compare")
    {
        for (const std::string& p : {std::string("gbn"), std::string("sr")})
        {
            for (uint32_t w : {1u, 2u, 4u, 8u, 16u, 32u})
            {
                ArqConfig c = base;
                c.experiment = "p3_compare";
                c.protocol = p;
                c.delayMs = 5.0;
                c.lossProbability = 0.05;
                c.window = w;
                c.receiverBuffer = 20;
                RunAndWrite(out, c);
            }
        }
        if (mode == "sr-compare") return 0;
    }

    if (mode == "all" || mode == "sr-buffer")
    {
        for (uint32_t b : {4u, 8u, 16u, 32u, 64u})
        {
            ArqConfig c = base;
            c.experiment = "p3_buffer";
            c.protocol = "sr";
            c.delayMs = 5.0;
            c.lossProbability = 0.05;
            c.window = 32;
            c.receiverBuffer = b;
            RunAndWrite(out, c);
        }
        if (mode == "sr-buffer") return 0;
    }

    if (mode != "all")
    {
        std::cerr << "Unknown mode: " << mode << '\n';
        return 2;
    }

    out.close();
    std::cout << "\nWrote results to " << outFile << '\n';
    return 0;
}
