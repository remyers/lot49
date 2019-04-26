#include "Utils.hpp"
#include "MeshNode.hpp"
#include <iterator>

using namespace lot49;

namespace lot49 {

std::ostream& operator<< (std::ostream& out, ETransactionType inType)
{
    switch (inType)
    {
        case ETransactionType::eIssue : return out << "eIssue" ;
        case ETransactionType::eTransfer: return out << "eTransfer";
        case ETransactionType::eSetup: return out << "eSetup";
        case ETransactionType::eRefund: return out << "eRefund";
        case ETransactionType::eUpdateAndSettle: return out << "eUpdateAndSettle";
        case ETransactionType::eClose: return out << "eClose";
        // omit default case to trigger compiler warning for missing cases
    };
    return out << static_cast<std::uint16_t>(inType);
}

std::ostream& operator<< (std::ostream& out, EChannelState inType)
{
    switch (inType)
    {
        case EChannelState::eSetup1 : return out << "eSetup1" ;
        case EChannelState::eSetup2 : return out << "eSetup2";
        case EChannelState::eNegotiate1: return out << "eNegotiate1";
        case EChannelState::eNegotiate2: return out << "eNegotiate2";
        case EChannelState::eNegotiate3: return out << "eNegotiate3";
        case EChannelState::eReceipt1: return out << "eReceipt1";
        case EChannelState::eReceipt2: return out << "eReceipt2";
        case EChannelState::eClose1: return out << "eClose1";
        case EChannelState::eClose2: return out << "eClose2";
        // omit default case to trigger compiler warning for missing cases
    };
    return out << static_cast<std::uint16_t>(inType);
}

std::ostream &operator<<(std::ostream &out, const std::vector<HGID> &v) {
    out << std::hex;
    for (auto hgid : v) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(hgid);
    }
    return out;
}

double Distance(const std::pair<double, double>& inA, const std::pair<double, double>& inB)
{
    // otherwise, move towards waypoint
    const double& x1 = inA.first;
    const double& y1 = inA.second;
    const double& x2 = inB.first;
    const double& y2 = inB.second;

    double distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    return distance;
}

std::ostream &operator<<(std::ostream &out, const MeshNode &n)
{
    out << "HGID: " << std::hex << std::setw(4) << std::setfill('0') << n.GetHGID() << std::endl;
    out << "\tptr = 0x" << std::hex << &n << std::endl;
    out << "\tPending Channel Node: " << n.mPendingChannelNode << std::endl;
    out << "\tGateway: " << (n.mIsGateway ? "true" : "false") << std::endl;
    out << "\tCorrespondent: " << n.mCorrespondent;
    out << " (distance = " << Distance(n.mCurrentPos, MeshNode::FromHGID(n.mCorrespondent).mCurrentPos) << ")" << std::endl;
    out << "\tPublic Key: " << n.GetPublicKey() << std::endl;
    out << "\tSeed: ";
    out << std::hex;
    for (auto byte : n.mSeed ) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    return out;
}

std::ostream &operator<<(std::ostream &out, const L49Header &i)
{
    out << "Incentive, Type: " << i.mType << " Prepaid Tokens: " << (int) i.mPrepaidTokens  << (i.mWitness ? " (Witness) " : "-") << std::endl;
    out << "\tRelay Path: [";
    out << std::hex;
    for (auto  byte : i.mRelayPath) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    out << "]" <<  std::endl;
    out << "\tSignature: " << bls::Signature::FromBytes(i.mSignature.data()) << std::endl;
    /*
    out << "\tSignature: ";
    out << std::hex;
    for (auto  byte : i.mSignature) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    out <<  std::endl;
    */
   return out;
}

std::ostream &operator<<(std::ostream &out, const MeshMessage &m)
{
    out << "Message,  " << m.mIncentive.mType << ", Prepaid Tokens: " << (int) m.mIncentive.mPrepaidTokens << ", ";
    out << std::setw(2) << std::setfill('0') << m.mSource << " >> ";
    out << std::setw(2) << std::setfill('0') << m.mSender << " -> ";
    out << std::setw(2) << std::setfill('0') << m.mReceiver << " >> ";
    out << std::setw(2) << std::setfill('0') << m.mDestination << ".";
    if (m.mPayloadData.empty()) {
        out << " Payload: " << std::endl << "\t[]" << std::endl;
    }
    else if (m.mIncentive.mWitness) {
        MeshMessage witness_message;
        witness_message.FromBytes(m.mPayloadData);
        out << " Payload: Witness: " << std::endl << "\t\t[" << witness_message << "]" << std::endl;
    }
    else {
        std::string payload_text(reinterpret_cast<const char*>(m.mPayloadData.data()), m.mPayloadData.size());
        out << " Payload: " << std::endl << "\t\t[" << payload_text << "]" << std::endl;
    }
    out << "\t\t" << m.mIncentive;
    return out;
}

std::ofstream sLogfile;
std::ofstream& LOG() {
    std::string filename = MeshNode::sParametersString + std::string("lot49.log");
    if (!sLogfile.is_open()) {
        sLogfile.open(filename);
    }
    return sLogfile;
}

std::ofstream sStatsfile;
std::ofstream& STATS() {
    if (!sStatsfile.is_open()) {
        std::string filename = MeshNode::sParametersString + "lot49_stats.csv";
        sStatsfile.open(filename);
        sStatsfile << "time, label, sender, receiver, source, destination, distance, incentive type, prepaid tokens, relay path size, agg signature size, is witness, payload data size, receiver unspent tokens, receiver channel state, receiver channel confirmed" << std::endl;
    }
    return sStatsfile;
}

std::ofstream sTopology;
std::ofstream& TOPOLOGY() {
    if (!sTopology.is_open()) {
        std::string filename = MeshNode::sParametersString + "lot49_topology.csv";
        sTopology.open(filename);
        sTopology << "time, node, correspondent, distance, current_x, current_y, paused, next_node, in_channels, out_channels, received_tokens, spent_tokens, token_balance" << std::endl;
    }
    return sTopology;
};

void CloseLogs()
{
    sLogfile.close();
    sStatsfile.close();
    sTopology.close();
}

std::vector<uint8_t> HexToBytes(const std::string& hex) {
  std::vector<uint8_t> bytes;

  for (unsigned int i = 0; i < hex.length(); i += 2) {
    std::string byteString = hex.substr(i, 2);
    char byte = (uint8_t) strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
  }

  return bytes;
}

}; // namespace lot49