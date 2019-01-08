
#include "MeshNode.hpp"
#include <random>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <fstream>

using namespace std;
using namespace lot49;

//
// simulation parameters
//

static double sGatewayPercent = 0.2; // percent of nodes that are also internet gateways
static double sMaxSize = 5000; // meters width
static double sMoveRate = 85; // meters per minute
static int sPauseTime = 5; // minutes of simulation
static int sCurrentTime = 0; // minutes of simulation
static int sPayloadSize = 50; // bytes
static int sRadioRange = 800; // radio communication range in meters

std::vector<lot49::MeshNode> lot49::MeshNode::sNodes;
std::list<lot49::MeshRoute> lot49::MeshNode::sRoutes;

static std::default_random_engine rng(std::random_device{}());
static std::uniform_int_distribution<uint8_t> dist(0, 255); //(min, max)

//get one
const double random_num = dist(rng);

namespace lot49
{

static uint16_t COMMITTED_TOKENS = 500;
static uint8_t PREPAID_TOKENS = 10;

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
    out << "HGID: " << std::hex << std::setw(4) << std::setfill('0') << n.GetHGID() << endl;
    out << "\tptr = 0x" << std::hex << &n << endl;
    out << "\tWitness: " << n.mWitnessNode << endl;
    out << "\tGateway: " << (n.mIsGateway ? "true" : "false") << endl;
    out << "\tCorrespondent: " << n.mCorrespondent;
    out << " (distance = " << Distance(n.mCurrentPos, MeshNode::FromHGID(n.mCorrespondent).mCurrentPos) << ")" << endl;
    out << "\tPublic Key: " << n.GetPublicKey() << endl;
    out << "\tSeed: ";
    out << std::hex;
    for (auto byte : n.mSeed ) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    return out;
}

std::ostream& operator<<(std::ostream &out, const L49Header &i)
{
    out << "Incentive, Type: " << i.mType << " Prepaid Tokens: " << (int) i.mPrepaidTokens  << (i.mWitness ? " (Witness) " : "-") << endl;
    out << "\tRelay Path: [";
    out << std::hex;
    for (auto  byte : i.mRelayPath) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    out << "]";
    /*
    out << "\tSignature: ";
    out << std::hex;
    for (auto  byte : i.mSignature) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
    out << endl;
    */
   return out;
}

std::ostream &operator<<(std::ostream &out, const MeshMessage &m)
{
    out << "Message, Source: " << std::setw(2) << std::setfill('0') << m.mSource;
    out << " Sender: " << std::setw(2) << std::setfill('0') << m.mSender;
    out << " Receiver: " << std::setw(2) << std::setfill('0') << m.mReceiver;
    out << " Destination: " << std::setw(2) << std::setfill('0') << m.mDestination;
    if (m.mPayloadData.empty()) {
        out << " Payload: " << endl << "\t[]" << endl;
    }
    else if (m.mIncentive.mWitness) {
        MeshMessage witness_message;
        witness_message.FromBytes(m.mPayloadData);
        out << " Payload: Witness: " << endl << "\t[" << witness_message << "]" << endl;
    }
    else {
        std::string payload_text(reinterpret_cast<const char*>(m.mPayloadData.data()), m.mPayloadData.size());
        out << " Payload: " << endl << "\t[" << payload_text << "]" << endl;
    }
    out << "\t" << m.mIncentive;
    return out;
}

static std::ofstream sLogfile;
std::ofstream& LOG() {
    if (!sLogfile.is_open()) {
        sLogfile.open("lot49.log");
    }
    return sLogfile;
};
#define _log LOG()

static std::ofstream sStatsfile;
std::ofstream& STATS() {
    if (!sStatsfile.is_open()) {
        sStatsfile.open("lot49_stats.csv");
        sStatsfile << "time, label, sender, receiver, source, destination, distance, incentive type, prepaid tokens, relay path size, agg signature size, is witness, payload data size, receiver unspent tokens, receiver channel state, receiver channel confirmed" << endl;
    }
    return sStatsfile;
};
#define _stats STATS()

void MeshNode::WriteStats(const std::string& inLabel, const lot49::MeshMessage& inMessage)
{
    MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);
    MeshNode& receiver = MeshNode::FromHGID(inMessage.mReceiver);
    double tx_distance = Distance(sender.mCurrentPos, receiver.mCurrentPos);
 
    // time
    _stats << sCurrentTime << ", ";
    // label
    _stats << inLabel << ", ";
    // sender
    _stats << std::hex << setw(4) << (int) inMessage.mSender << ", ";    
    // receiver
    _stats << std::hex << setw(4) << (int) inMessage.mReceiver << ", ";
    // source
     _stats << std::hex << setw(4) << (int) inMessage.mSource << ", ";   
    // destination
    _stats << std::hex << setw(4) << (int) inMessage.mDestination << ", ";
    // distance
    _stats << std::dec << (int) tx_distance << ", ";
    // incentive_type
    _stats << inMessage.mIncentive.mType << ", ";
    // prepaid_tokens
    _stats << std::dec << (int) inMessage.mIncentive.mPrepaidTokens << ", ";
    // relay_path_size
    _stats << std::dec << inMessage.mIncentive.mRelayPath.size() << ", ";
    // agg_signature_size
    _stats << std::dec << inMessage.mIncentive.mSignature.size() << ", ";
    // is_witness
    _stats << (inMessage.mIncentive.mWitness ? "witness" : (inMessage.mIncentive.mType == eSetup1 ? "setup" : "payload")) << ", ";
    // payload_data_size
    _stats << std::dec << inMessage.mPayloadData.size() << ", ";
    // receiver_unspent_tokens, receiver_channel_state, receiver_channel_confirmed
    if (receiver.HasChannel(inMessage.mReceiver, inMessage.mSender)) {
        PeerChannel& channel = receiver.GetChannel(inMessage.mReceiver, inMessage.mSender);
        _stats << std::dec << channel.mUnspentTokens << ", ";
        _stats << std::dec << channel.mState << ", ";
        _stats << (channel.mConfirmed ? "true" : "false") << endl;
    }
    else {
        _stats << std::dec << "-, -, -" << endl;
    }
}

// create mesh nodes
void MeshNode::CreateNodes(const int inCount)
{
    sNodes.resize(inCount); 

    // default witness node
    HGID witness_node = 0xFFFF;

    //std::generate(sNodes.begin(), sNodes.end(), [&] {return MeshNode();});

    // set the last node as a gateway for verifying setup transactions
    int num_gateways = sNodes.size() * sGatewayPercent;
    
    // each node corresponds with one other node
    for (int i = 0; i < inCount; i++) {
        MeshNode& n = MeshNode::FromIndex(i);
        n.SetWitnessNode(witness_node);
        HGID correspondent_node = MeshNode::FromIndex((i+1) % inCount).GetHGID();
        n.SetCorrespondentNode(correspondent_node);

        if (i < num_gateways) {
            n.mIsGateway = true;
        }

        _log << n;
        _log << endl;
    }    
}

MeshNode &MeshNode::FromIndex(const int inIndex)
{
    if (inIndex > sNodes.size())
    {
        CreateNodes(inIndex + 1);
    }
    return sNodes[inIndex];
}

//  Lookup a node from a Hashed GID
MeshNode &MeshNode::FromHGID(const HGID &inHGID)
{
    for (auto node = sNodes.begin(); node != sNodes.end(); ++node)
    {
        if (node->GetHGID() == inHGID) {
            return *node;
        }
    }
    throw std::invalid_argument("invalid HGID");
}

// Lookup a node from a public key
MeshNode &MeshNode::FromPublicKey(const bls::PublicKey& inPk)
{
    for (auto node = sNodes.begin(); node != sNodes.end(); ++node)
    {
        if (node->GetPublicKey() == inPk) {
            return *node;
        }
    }
    throw std::invalid_argument("invalid Public Key");
}

void MeshNode::ClearRoutes() {
    sRoutes.clear();
}

// recursively find shortest route to a node
bool MeshNode::FindRoute(const HGID inDestination, MeshRoute& ioRoute, std::list<HGID>& ioVisited, double& ioDistance) 
{
    // at destination node
    if (inDestination == GetHGID()) {
        ioRoute.clear();
        ioRoute.push_back(GetHGID());
        return true;
    }

    ioVisited.push_back(GetHGID());

    double min_distance = std::numeric_limits<double>::max();
    MeshRoute min_route;
    bool found = false;
    for ( auto& n : sNodes) {
        // skip self
        if (n.GetHGID() == GetHGID()) {
            continue;
        }
        // no loops, skip routes that already include this node
        if (std::find(ioVisited.begin(), ioVisited.end(), n.GetHGID()) != ioVisited.end()) {
            continue;
        }
        // find shortest distance to destination from nodes within radio range
        double radio_range = Distance(mCurrentPos, n.mCurrentPos);
        if (radio_range < sRadioRange) {
            MeshRoute route;
            double distance = 0;
            if (n.FindRoute(inDestination, route, ioVisited, distance) && (distance + radio_range) < min_distance) {
                route.insert(route.begin(), GetHGID());
                min_route = route;
                min_distance = distance + radio_range;
                found = true;
            }
        }
    }

    // remove current node from visited list
    ioVisited.pop_back();

    if (found) {
        ioDistance = min_distance;
        ioRoute = min_route;
    }
    return found;
}

/*
// recursively find shortest route to a node
bool MeshNode::FindRoute(const HGID inDestination, MeshRoute& ioRoute, std::list<HGID>& ioVisited, double& ioDistance) 
{
    // at destination node
    if (inDestination == GetHGID()) {
        ioRoute.push_back(GetHGID());
        return true;
    }

    ioVisited.push_back(GetHGID());

    double min_distance = std::numeric_limits<double>::max();
    MeshRoute min_route;
    bool found = false;
    for ( auto& n : sNodes) {
        // skip self
        if (n.GetHGID() == GetHGID()) {
            continue;
        }
        // no loops, skip routes that already include this node
        if (std::find(ioVisited.begin(), ioVisited.end(), n.GetHGID()) != ioVisited.end()) {
            continue;
        }
        // find shortest distance to destination from nodes within radio range
        double distance = Distance(mCurrentPos, n.mCurrentPos);
        if (distance < sRadioRange) {
            MeshRoute route;
            route.push_back(GetHGID());
            distance += ioDistance;
            if (distance < min_distance && n.FindRoute(inDestination, route, ioVisited, distance)) {
                route.insert(route.begin(), ioRoute.begin(), ioRoute.end());
                min_route = route;
                min_distance = distance;
                found = true;
            }
        }
    }

    if (found) {
        ioDistance = min_distance;
        ioRoute = min_route;
    }
    return found;
}
*/

HGID MeshNode::GetNextHop(HGID inNode, HGID inDestination)
{
    HGID next_hop;
    if (GetNextHop(inNode, inDestination, next_hop)) {
        return next_hop;
    }
    throw std::invalid_argument("No route to destination.");
}

bool MeshNode::GetNextHop(HGID inNode, HGID inDestination, HGID& outNextHop)
{
    // next hop along shortest breadth-first search route
    double distance = 0;
    MeshNode& node = MeshNode::FromHGID(inNode);
    MeshRoute route;
    std::list<HGID> visited;
    bool found = node.FindRoute(inDestination, route, visited, distance);
    if (found) {
        outNextHop = route[1];

        // only add route if not already added
        if (std::find(sRoutes.begin(), sRoutes.end(), route) == sRoutes.end()) {
            AddRoute(route);
        }
    }
    else {
        // use pre-set routes
        for (auto route = sRoutes.begin(); route != sRoutes.end(); ++route) {
            auto node_iter = std::find(route->begin(), route->end(), inNode);
            auto neighbor_iter = std::find(route->begin(), route->end(), inDestination);
            if (node_iter != route->end() && neighbor_iter != route->end()) {
                std::ptrdiff_t diff = std::distance(node_iter, neighbor_iter);
                assert(diff != 0);
                MeshRoute::iterator next_hop_iter = node_iter + (diff/abs(diff));
                outNextHop = *next_hop_iter;
                found = true;
            }
        }
    }

    return found;
}

// configure topology
void MeshNode::AddRoute(MeshRoute inRoute)
{
    // do not add if already added
    if (std::find(sRoutes.begin(), sRoutes.end(), inRoute) != sRoutes.end()) {
        return;
    }
    sRoutes.push_back(inRoute);

    // propose channels with neighbors (forward)
    auto route_end = inRoute.begin() + (inRoute.size() - 1);
    for (auto hgid_iter = inRoute.begin(); hgid_iter != route_end; ++hgid_iter) {
        //_log << "Node " << std::hex << *hgid_iter << ", Neighbor " << *(hgid_iter+1) << endl;
        FromHGID(*hgid_iter).ProposeChannel(*(hgid_iter+1));
    }

    // propose channels with neighbors (backwards)
    auto route_rend = inRoute.rbegin() + (inRoute.size() - 1);
    for (auto hgid_iter = inRoute.rbegin(); hgid_iter != route_rend; ++hgid_iter) {
        //_log << "Node: " << std::hex << *hgid_iter << " Neighbor: " << *(hgid_iter+1) << endl;
        FromHGID(*hgid_iter).ProposeChannel(*(hgid_iter+1));
    }
}

bool MeshNode::HasNeighbor(HGID inNode, HGID inNeighbor)
{
    bool found = false;

    // if no simulated coordinates, check routes
    for (auto route = sRoutes.begin(); !found && route != sRoutes.end(); ++route)
    {
        auto node_iter = std::find(route->begin(), route->end(), inNode);
        auto neighbor_iter = std::find(route->begin(), route->end(), inNeighbor);
        if (node_iter != route->end() && neighbor_iter != route->end()) {
            found = (abs(std::distance(node_iter, neighbor_iter)) == 1);
        }
    }
    return found;
}

// ctor
MeshNode::MeshNode()
{
    static uint8_t sSeed = 0;
    mSeed.resize(32);
    // std::generate_n(mSeed.begin(), 32, [&] { return dist(rng); });

    // DEBUG
    std::fill(mSeed.begin(), mSeed.end(), sSeed++);
    
    // if no witness or correspondent set, then use same hgid as node
    mWitnessNode = GetHGID();
    mCorrespondent = GetHGID();

    mIsGateway = false;
 
    // initialize position and waypoint
    std::uniform_int_distribution<double> pos(-sMaxSize/2, sMaxSize/2);
    mWaypoint.first = pos(rng);
    mWaypoint.second = pos(rng);
    mCurrentPos.first = pos(rng);
    mCurrentPos.second = pos(rng);

    // only pause if at waypoint
    mPausedUntil = 0;
}

HGID MeshNode::GetHGID() const
{
    // TODO: use hash of public key, not first two seed values
    return *reinterpret_cast<const uint16_t *>(mSeed.data());
}

// access private key
const bls::PrivateKey MeshNode::GetPrivateKey() const
{
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    return sk;    
}

// access public key
const bls::PublicKey MeshNode::GetPublicKey() const
{
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    return sk.GetPublicKey();
}

// initiate a payment channel if one doesn't already exist with this neighbor
void MeshNode::ProposeChannel(HGID inNeighbor)
{
    if (!HasNeighbor(GetHGID(), inNeighbor)) {
        assert(0);
        return;
    }

    // has this node already proposed a channel to inNeighbor?
    if (MeshNode::FromHGID(inNeighbor).HasChannel(GetHGID(), inNeighbor)) {
        return;
    }

    _log << "Node " << GetHGID() << ", ";
    _log << "ProposeChannel to " << inNeighbor << endl;
    _log << "Proposing Peer: " << GetHGID() << endl;
 
    if (HasChannel(GetHGID(), inNeighbor)) {
        assert(0);
    }
    else {
        PeerChannel theChannel;
        theChannel.mFundingPeer = inNeighbor;
        theChannel.mProposingPeer = GetHGID();
        theChannel.mUnspentTokens = COMMITTED_TOKENS;
        theChannel.mSpentTokens = 0;
        theChannel.mLastNonce = 0;
        theChannel.mState = eSetup1;
        theChannel.mConfirmed = false;
    
        mPeerChannels[make_pair(theChannel.mProposingPeer, theChannel.mFundingPeer)] = theChannel;
    }

    MeshMessage theMessage;
    theMessage.mSource = inNeighbor;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inNeighbor;
    theMessage.mDestination = inNeighbor;
        
    // initialize incentive aggregate signature by signing refund tx
    theMessage.mIncentive.mType = eSetup1;
    theMessage.mIncentive.mPrepaidTokens = 0;

    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(theMessage);
    bls::Signature refund_sig = SignTransaction(theImpliedTransactions.front());   

    theMessage.mIncentive.mSignature.resize(bls::Signature::SIGNATURE_SIZE);
    refund_sig.Serialize(&theMessage.mIncentive.mSignature[0]);

    WriteStats("Propose_Channel", theMessage);
    SendTransmission(theMessage);
}

// return true if channel exists with this neighbor
bool MeshNode::HasChannel(HGID inProposer, HGID inFunder) const
{
    return (mPeerChannels.find(make_pair(inProposer, inFunder)) != mPeerChannels.end());
}

// get existing channel
PeerChannel& MeshNode::GetChannel(HGID inProposer, HGID inFunder)
{
    auto channel_iter = mPeerChannels.find(make_pair(inProposer, inFunder));
    if (channel_iter == mPeerChannels.end()) {
        throw std::invalid_argument("No channel exists for neighbor.");
    }
    return channel_iter->second;
}

void SavePayloadHash(PeerChannel& ioChannel, const std::vector<uint8_t>& inData)
{
    ioChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&(ioChannel.mPayloadHash[0]), reinterpret_cast<const uint8_t*>(inData.data()), inData.size()); 
    _log << "\tSave payload hash(" << std::hex << ioChannel.mProposingPeer << ", " << ioChannel.mFundingPeer << "): [";
    for (int v: ioChannel.mPayloadHash) { _log << std::hex << v; }
    _log << "] ";
    _log << endl;   
} 

void SaveWitnessHash(PeerChannel& ioChannel, const std::vector<uint8_t>& inData)
{
    ioChannel.mWitnessHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&(ioChannel.mWitnessHash[0]), reinterpret_cast<const uint8_t*>(inData.data()), inData.size()); 
    _log << "\tSave witness hash(" << std::hex << ioChannel.mProposingPeer << ", " << ioChannel.mFundingPeer << "): [";
    for (int v: ioChannel.mWitnessHash) { _log << std::hex << v; }
    _log << "] ";
    _log << endl;   
} 

// originate new message
bool MeshNode::OriginateMessage(const HGID inDestination, const std::vector<uint8_t>& inPayload)
{
    std::string payload_text(reinterpret_cast<const char*>(inPayload.data()), inPayload.size());
    _log << "Node " << GetHGID() << ", ";
    _log << "OriginateMessage, Destination: " << inDestination << ", Payload: [" << payload_text << "]" << endl << endl;

    HGID next_hop;
    if (!GetNextHop(GetHGID(), inDestination, next_hop)) {
        _log << "!! No route found !!" << endl;
        return false;
    }

    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = next_hop;
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inDestination;
    theMessage.mPayloadData = inPayload;

    PeerChannel &theChannel = GetChannel(theMessage.mReceiver, GetHGID());

    assert(theChannel.mState == eSetup2 || theChannel.mState == eReceipt2 || theChannel.mState == eReceipt1);

   if (theChannel.mUnspentTokens < PREPAID_TOKENS) {
         _log << "!! insufficient funds, unspent tokens = " << theChannel.mUnspentTokens << " !!" << endl;
        return false;       
    }

    theChannel.mUnspentTokens -= PREPAID_TOKENS;
    theChannel.mSpentTokens += PREPAID_TOKENS;
    theChannel.mLastNonce += 1;
    theChannel.mState = (theMessage.mDestination == theMessage.mReceiver) ? eNegotiate2 : eNegotiate1;
    
    theMessage.mIncentive.mWitness = false;
    theMessage.mIncentive.mPrepaidTokens = PREPAID_TOKENS;
    theMessage.mIncentive.mSignature = theChannel.mRefundSignature;
    theMessage.mIncentive.mType = theChannel.mState;

    // save a local copy of the payload hash for confirming receipt2 messages
    assert(theMessage.mIncentive.mType < eReceipt1 );
    SavePayloadHash(theChannel, theMessage.mPayloadData);

    UpdateIncentiveHeader(theMessage);

    WriteStats("Originate_Message", theMessage);
    SendTransmission(theMessage);
    return true;
}

// relay a message
void MeshNode::RelayMessage(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "RelayMessage: " << inMessage << endl;

    // confirm setup transaction on the blockchain
    PeerChannel &theSenderChannel = GetChannel(GetHGID(), inMessage.mSender);
    if (theSenderChannel.mConfirmed == false) {
        HGID gateway;
        if (!GetNearestGateway(gateway)) {
            _log << "!! No route to gateway !! " << endl;    
        }
        else {
            theSenderChannel.mConfirmed = ConfirmSetupTransaction(inMessage, gateway);
        }
    }

    // receive payment from sender
    uint8_t received_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    theSenderChannel.mUnspentTokens -= received_tokens;
    theSenderChannel.mSpentTokens += received_tokens;
    theSenderChannel.mLastNonce += 1;
    theSenderChannel.mState = inMessage.mIncentive.mType;

    HGID next_hop = GetNextHop(GetHGID(), inMessage.mDestination);

    // pay next hop
    uint8_t spent_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size()) - 1;
    PeerChannel &theChannel = GetChannel(next_hop, GetHGID());
    theChannel.mUnspentTokens -= spent_tokens;
    theChannel.mSpentTokens += spent_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = inMessage.mIncentive.mType;
    
    // save a local copy of the payload hash for confirming receipt2 messages
    assert ( inMessage.mIncentive.mType < eReceipt1 );
    if (!inMessage.mIncentive.mWitness) {
        SavePayloadHash(theChannel, inMessage.mPayloadData);
    }
    else {
        SaveWitnessHash(theChannel, inMessage.mPayloadData);
    }

    // new relay message
    MeshMessage outMessage = inMessage;
    outMessage.mSender = GetHGID();
    outMessage.mReceiver = next_hop;

    if (outMessage.mReceiver == outMessage.mDestination && outMessage.mIncentive.mType == eNegotiate1 ) {
        // next node is destination node
        outMessage.mIncentive.mType = eNegotiate2;
        theChannel.mState = eNegotiate2;
    }

    UpdateIncentiveHeader(outMessage);

    WriteStats("Relay_Message", outMessage);

    // send message to next hop
    SendTransmission(outMessage);
}

// fund a channel
void MeshNode::FundChannel(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "FundChannel: " << inMessage << endl;
    _log << "Proposing Peer: " << inMessage.mSender << endl;
    
    // TODO: check that funds exist, etc.

    if (HasChannel(inMessage.mSender, GetHGID())) {
        assert(0);
    }
    else {
        // create channel entry for peer that proposed the channel
        PeerChannel theChannel;
        theChannel.mFundingPeer = GetHGID();
        theChannel.mProposingPeer = inMessage.mSender;
        theChannel.mUnspentTokens = COMMITTED_TOKENS; // always commit default amount when funding a channel
        theChannel.mSpentTokens = 0;
        theChannel.mLastNonce = 0;
        theChannel.mState = eSetup2;
        theChannel.mRefundSignature = inMessage.mIncentive.mSignature;
        theChannel.mConfirmed = true;

        // save a local copy of the payload hash for confirming receipt2 messages
        assert ( inMessage.mIncentive.mType < eReceipt1 );
        SavePayloadHash(theChannel, inMessage.mPayloadData);           
   
        mPeerChannels[make_pair(theChannel.mProposingPeer, theChannel.mFundingPeer)] = theChannel;
    }
}

// receive message
void MeshNode::ReceiveMessage(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "ReceiveMessage: " << inMessage << endl;

    // confirm setup transaction on the blockchain
    PeerChannel &theSenderChannel = GetChannel(GetHGID(), inMessage.mSender);
    if (theSenderChannel.mConfirmed == false) {
        HGID gateway;
        if (!GetNearestGateway(gateway)) {
            _log << "!! No route to gateway !! " << endl;    
        }
        else {
            theSenderChannel.mConfirmed = ConfirmSetupTransaction(inMessage, gateway);
        }
    }

    // receive remaining tokens from sender
    uint8_t remaining_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    theSenderChannel.mUnspentTokens -= remaining_tokens;
    theSenderChannel.mSpentTokens += remaining_tokens;
    theSenderChannel.mLastNonce += 1;
    theSenderChannel.mState = inMessage.mIncentive.mType;

    // channel for sending return receipt
    PeerChannel& theReturnChannel = GetChannel(inMessage.mSender, GetHGID());
    theReturnChannel.mLastNonce += 1;
    theReturnChannel.mState = eReceipt1;
 
    // save a local copy of the payload hash for confirming receipt2 messages
    if (!inMessage.mIncentive.mWitness) {
        SavePayloadHash(theReturnChannel, inMessage.mPayloadData);
    }
    else {
        SaveWitnessHash(theReturnChannel, inMessage.mPayloadData);
    }
    
    // message received and marked for signing witness node ? 
    if (inMessage.mIncentive.mWitness) {
        // verify the setup transaction on the blockchain
        MeshMessage witness_message;
        witness_message.FromBytes(inMessage.mPayloadData);
        std::vector<ImpliedTransaction> theTransactions = GetTransactions(witness_message);
        if (!VerifyMessage(witness_message)) {
            assert(0);
            return;
        }

        _log << "Node " << std::setw(4) << std::setfill('0') << inMessage.mReceiver << ": confirmed setup transaction:" << endl << "\t" << witness_message << endl;
        cout << "Node " << std::setw(4) << std::setfill('0') << inMessage.mReceiver << ": confirmed setup transaction:" << endl << "\t" << witness_message << endl;

        // get public key of Setup transaction
        assert(theTransactions[2].GetType() == eSetup);
        bls::PublicKey pk = theTransactions[1].GetSigner();
        // TODO: check that the setup tx signer has the required balance
        // TODO: commit the transaction chain to the blockchain
    }
    else {
        std::string payload_text(reinterpret_cast<const char*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());
        _log << "Node " << std::setw(4) << std::setfill('0') << inMessage.mReceiver << " received message: [" << payload_text << "] !" << endl;
        cout << "Node " << std::setw(4) << std::setfill('0') << inMessage.mReceiver << " received message: [" << payload_text << "] !" << endl;
    }
     
    // send return receipt
    MeshMessage theMessage = inMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inMessage.mSender;
    theMessage.mIncentive.mType = eReceipt1;
    UpdateIncentiveHeader(theMessage);

    // no need to send payload, hash is cached by nodes
    theMessage.mPayloadData.clear();

    WriteStats("Send_Receipt", theMessage);

    // send proof of receipt to previous hop
    SendTransmission(theMessage);
}

// receive delivery receipt
void MeshNode::RelayDeliveryReceipt(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "RelayDeliveryReceipt: " << inMessage << endl;

    // destination node confirms message hash matches
    if (!VerifyMessage(inMessage)) {
        _log << "\tVerifyMessage, failed!" << endl;
        return;
    }

    // TODO: process confirmed payment
    PeerChannel& theChannel = GetChannel(inMessage.mSender, GetHGID());

    // TODO: should update channel with payment here, when confirmed, not when relayed
    // auto pos = std::find(inMessage.mIncentive.mRelayPath.begin(), inMessage.mIncentive.mRelayPath.end(), GetHGID());
    // uint8_t earned_tokens = PREPAID_TOKENS - std::distance(inMessage.mIncentive.mRelayPath.begin(), pos);
    // theChannel.mUnspentTokens -= earned_tokens;
    // theChannel.mSpentTokens += earned_tokens;
    // theChannel.mLastNonce += 1;
    theChannel.mState = eReceipt2;    

    if (GetHGID() != inMessage.mSource) {
        // relay return receipt
        MeshMessage theMessage = inMessage;
        theMessage.mSender = GetHGID();
        theMessage.mReceiver = GetNextHop(GetHGID(), inMessage.mSource);

        // use cached hash of payload, do not resend payload with receipt
        theMessage.mPayloadData.clear();

        // message destination signs before relaying eReceipt1, all others just relay
        assert(theMessage.mIncentive.mType == eReceipt1 || theMessage.mIncentive.mType == eReceipt2);
        theMessage.mIncentive.mType = eReceipt2;
        // no need to call UpdateIncentiveHeader(theMessage) because receipts don't need any extra incentives

        WriteStats("Relay_Receipt", theMessage);

        // send proof of receipt to previous hop
        SendTransmission(theMessage);
    }
    else if (inMessage.mIncentive.mWitness) {
        _log << "Confirmation of channel setup received by relay " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from Witness Node " << std::setw(4) << inMessage.mDestination << "!" << endl;
        cout << "Confirmation of channel setup received by relay " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from Witness Node " << std::setw(4) << inMessage.mDestination << "!" << endl;
    }
    else {
        _log << "Delivery Receipt received by source " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from message destination " << std::setw(4) << inMessage.mDestination << "!" << endl;
        cout << "Delivery Receipt received by source " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from message destination " << std::setw(4) << inMessage.mDestination << "!" << endl;
    }
}

// confirm the setup transaction for a payment channel with a witness node (via inGateway) 
bool MeshNode::ConfirmSetupTransaction(const MeshMessage& inMessage, const HGID inGateway)
{
    _log << "Node " << GetHGID() << ", ";
    _log << "ConfirmSetupTransaction, Gateway: " << inGateway << ", Message Hash: [" << inMessage << "]" << endl << endl;

    if (GetHGID() == inGateway) {
        // TODO: handle special case when relaying through a gateway
        PeerChannel& theSenderChannel = GetChannel(inMessage.mSender, GetHGID());
        theSenderChannel.mConfirmed = true;
        return true;
    }

    HGID next_hop;
    if (!GetNextHop(GetHGID(), inGateway, next_hop)) {
        _log << "!! No route found !!" << endl;
        return false;
    }

    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = next_hop;
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inGateway;

    theMessage.mPayloadData = inMessage.Serialize();

    PeerChannel& theChannel = GetChannel(theMessage.mReceiver, GetHGID());

    assert(theChannel.mState == eSetup2 || theChannel.mState == eReceipt2 || theChannel.mState == eReceipt1);

    if (theChannel.mUnspentTokens < PREPAID_TOKENS) {
         _log << "!! insufficient funds, unspent tokens = " << theChannel.mUnspentTokens << " !!" << endl;
        return false;       
    }

    theChannel.mUnspentTokens -= PREPAID_TOKENS;
    theChannel.mSpentTokens += PREPAID_TOKENS;
    theChannel.mLastNonce += 1;
    theChannel.mState = (theMessage.mDestination == theMessage.mReceiver ? eNegotiate2 : eNegotiate1);
    
    theMessage.mIncentive.mWitness = true;
    theMessage.mIncentive.mPrepaidTokens = PREPAID_TOKENS;
    theMessage.mIncentive.mSignature = theChannel.mRefundSignature;
    theMessage.mIncentive.mType = theChannel.mState;

    // save a local copy of the payload hash for confirming receipt2 messages
    assert(theMessage.mIncentive.mType < eReceipt1 );
    SaveWitnessHash(theChannel, theMessage.mPayloadData);

    UpdateIncentiveHeader(theMessage);

    WriteStats("Send_Witness", theMessage);

    SendTransmission(theMessage);

    return true;
}


bls::Signature MeshNode::GetAggregateSignature(const MeshMessage& inMessage, const bool isSigning)
{
    const MeshNode& theSigningNode = MeshNode::FromHGID(inMessage.mSender);

    _log << "Node " << GetHGID() << ", ";
    _log << "GetAggregateSignature, " << inMessage << endl;

    // calculate aggregation info from implied transaction hashes and signing public keys  
    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(inMessage);
    vector<bls::Signature> sigs;
    std::vector<bls::AggregationInfo> aggregation_info;
    bls::PublicKey sender_pk = MeshNode::FromHGID(inMessage.mSender).GetPublicKey();
    bool isOtherSigner = !isSigning;
    bool isSkip = inMessage.mIncentive.mType < eReceipt1;
    for (auto tx = theImpliedTransactions.rbegin(); tx != theImpliedTransactions.rend(); tx++) {
        bls::PublicKey tx_signer_pk = tx->GetSigner();
        
        isSkip &= (tx_signer_pk != sender_pk);

        if (!isSkip) {
            // only add aggregation_info for previously aggregated signatures
            isOtherSigner |= tx_signer_pk != theSigningNode.GetPublicKey();

            if (isOtherSigner) {
                aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(tx_signer_pk, tx->GetTransactionHash().data()) );
            }
            else {
                // push current signature contained in the message
                sigs.push_back( theSigningNode.SignTransaction(*tx) );
            }
        }
        _log << "\tSigner: " << MeshNode::FromPublicKey(tx_signer_pk).GetHGID() << " Type: " << tx->GetType() << (isSkip ? "- " : (isOtherSigner ? "* " : " "));
        _log << " tx: [";
        for (int v: tx->GetTransactionHash()) { _log << std::hex << v; }
        _log << "] ";
        _log << endl;
    }

    // add aggregation info for destination's signature for payload message
    if (inMessage.mIncentive.mType >= eReceipt2 || (inMessage.mIncentive.mType == eReceipt1 && !isSigning)) {
        const MeshNode& destination = MeshNode::FromHGID(inMessage.mDestination);
        bls::PublicKey pk = destination.GetPublicKey();

        HGID direction_hgid = inMessage.mDestination;
        if (GetHGID() == direction_hgid) {
            // sending receipt back to source
            direction_hgid = inMessage.mSource;
        }
        HGID next_hop_hgid = GetNextHop(GetHGID(), direction_hgid);

        const PeerChannel& theChannel = GetChannel( next_hop_hgid, GetHGID());

        std::vector<uint8_t> hash = theChannel.mPayloadHash;
        if (inMessage.mIncentive.mWitness) {
            hash = theChannel.mWitnessHash;
        }

        _log << endl << "\tSigner: " << inMessage.mDestination << " Type: sign_payload* ("<< std::hex << theChannel.mProposingPeer << ", " << theChannel.mFundingPeer << ") ";
        _log << "hash: [";
        for (int v: hash) { _log << std::hex << v; }
        _log << "] ";
        _log << endl;

        aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(pk, hash.data()) );
    }    

    _log << endl;

    bls::AggregationInfo merged_aggregation_info = bls::AggregationInfo::MergeInfos(aggregation_info);
    bls::Signature agg_sig = bls::Signature::FromBytes(inMessage.mIncentive.mSignature.data());
    agg_sig.SetAggregationInfo(merged_aggregation_info);
    sigs.push_back(agg_sig);

    // message receiver signs the payload data
    if (inMessage.mIncentive.mType == eReceipt1 && isSigning) {
        assert(theSigningNode.GetHGID() == GetHGID());
        assert(inMessage.mSender == inMessage.mDestination);
        sigs.push_back( theSigningNode.SignMessage(inMessage.mPayloadData) );
    }

    // update aggregate signature
    try {
        agg_sig = bls::Signature::AggregateSigs(sigs);
    }
    catch (std::string &e) {
        _log << e << endl;
    }    

    return agg_sig;
}

// compute implied transaction from message and peer channel
std::vector<ImpliedTransaction> MeshNode::GetTransactions(const MeshMessage& inMessage)
{
    std::vector<ImpliedTransaction> theTransactions;
    const L49Header& incentive = inMessage.mIncentive;
    uint8_t prepaid_tokens = incentive.mPrepaidTokens;
    const MeshNode& destination = MeshNode::FromHGID(inMessage.mDestination);
    vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN, 0);
    const MeshNode& source = MeshNode::FromHGID(inMessage.mSource);
    const MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);

    HGID first_relay_hgid = inMessage.mDestination;
    if (!incentive.mRelayPath.empty()) {
        first_relay_hgid = incentive.mRelayPath.front();
    }
    if (inMessage.mSender == inMessage.mSource) {
        first_relay_hgid = inMessage.mReceiver;
    }
    else if (inMessage.mReceiver == inMessage.mSource) {
        first_relay_hgid = inMessage.mSender;
    }
    const MeshNode& first_relay = MeshNode::FromHGID(first_relay_hgid);

    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

    ImpliedTransaction issued_value_tx = ImpliedTransaction::Issue(source.GetPublicKey(), 255);
    ImpliedTransaction setup_tx = ImpliedTransaction::Setup(issued_value_tx, source.GetPublicKey(), first_relay.GetPublicKey(), COMMITTED_TOKENS);

    _log << "GetTransactions, Type: " << incentive.mType << endl;

    if (incentive.mType >= eSetup1) {
        // create refund tx, first relay signs and then source signs
        theTransactions.push_back( ImpliedTransaction::Refund(setup_tx, source.GetPublicKey(), first_relay.GetPublicKey(), first_relay.GetPublicKey(), COMMITTED_TOKENS) );
    }
    if (incentive.mType >= eSetup2) {
        // create funding tx
        theTransactions.push_back( ImpliedTransaction::Refund(setup_tx, source.GetPublicKey(), first_relay.GetPublicKey(), source.GetPublicKey(), COMMITTED_TOKENS) );
        theTransactions.push_back( setup_tx );
    }
    if (incentive.mType >= eNegotiate1) {       
        // append current node to path
        vector<HGID> path = incentive.mRelayPath;

        if (incentive.mType < eNegotiate2 ) {
            path.push_back(inMessage.mReceiver);
        }

        ImpliedTransaction last_update_tx = setup_tx;
        HGID hgid1 = inMessage.mSource;
        for (auto hgid2 = path.begin(); hgid2 != path.end(); ++hgid2) {
            const MeshNode& sender = MeshNode::FromHGID(hgid1);
            const MeshNode& receiver = MeshNode::FromHGID(*hgid2);
            // create update and settle tx, sender signs and then receiver signs
            theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(last_update_tx, sender.GetPublicKey(), receiver.GetPublicKey(), sender.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
            theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(last_update_tx, sender.GetPublicKey(), receiver.GetPublicKey(), receiver.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
            
            _log << endl << "\t\t public key: " << sender.GetPublicKey() << ", tokens: " << (int) prepaid_tokens << ", sender:" << sender.GetHGID() << ", receiver:" << receiver.GetHGID() << " tx: [";
            for (int v: theTransactions.back().GetTransactionHash()) { _log << std::hex << v; }
            _log << "] " << endl;

            hgid1 = *hgid2;
            prepaid_tokens--;
            last_update_tx = theTransactions.back();
        }
    }
    if (incentive.mType >= eNegotiate2) {

        // last incentive is from penultimate relay to destination
        HGID penultimate_node = incentive.mRelayPath.empty() ? inMessage.mSource : incentive.mRelayPath.back();
        const MeshNode& sender = MeshNode::FromHGID(penultimate_node);
        const MeshNode& receiver = MeshNode::FromHGID(inMessage.mDestination);
        // create update and settle tx for delivery to destination node, sender signs and then destination signs
        theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(theTransactions.back(), sender.GetPublicKey(), receiver.GetPublicKey(), sender.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
        theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(theTransactions.back(), sender.GetPublicKey(), receiver.GetPublicKey(), destination.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
    }
    
    return theTransactions;
}

void MeshNode::UpdateIncentiveHeader(MeshMessage& ioMessage)
{
    _log << "Node " << GetHGID() << ", ";
    _log << "UpdateIncentiveHeader " << ioMessage << endl;

    L49Header &incentive = ioMessage.mIncentive;
    const PeerChannel &theChannel = GetChannel(ioMessage.mReceiver, GetHGID());

    incentive.mType = theChannel.mState;

    // no need to update the incentive header when relaying delivery receipt
    assert(incentive.mType != eReceipt2);

    if (incentive.mSignature.empty()) {
        throw std::invalid_argument("invalid MeshMessage: L49Header is missing an aggregate signature");
    }    
    
    if ((theChannel.mState == eNegotiate1 || theChannel.mState == eNegotiate2) && ioMessage.mSender != ioMessage.mSource)
    {
        // add relay node to path
        incentive.mRelayPath.push_back(ioMessage.mSender);
    }

    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(ioMessage);
    bls::Signature agg_sig = GetAggregateSignature(ioMessage, true);    

    agg_sig.Serialize(&incentive.mSignature[0]);
}

//
bls::Signature MeshNode::SignTransaction(const ImpliedTransaction& inTransaction) const
{
    _log << "Node " << GetHGID() << ", ";
    _log << "SignTransaction, Tx Type: " << inTransaction.GetType() << " tx: [";
    for (int v: inTransaction.GetTransactionHash()) { _log << std::hex << v; } 
    _log << "]" << endl;
    _log << "\tTx Signer PK: ";
    _log << inTransaction.GetSigner();
    _log << endl;
 
    std::vector<uint8_t> msg = inTransaction.Serialize();
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::PublicKey pk = sk.GetPublicKey();
    bls::Signature sig = sk.Sign(msg.data(), msg.size());

    //_log << "\tSignature: " << sig << endl;
    return sig;
}

bls::Signature MeshNode::SignMessage(const std::vector<uint8_t>& inPayload) const
{
    //std::string theTextPayload(reinterpret_cast<const char*>(inPayload.data()), inPayload.size());
    _log << "Node " << GetHGID() << ", SignMessage: size = " << std::dec << inPayload.size() << " [";
    for (int v: inPayload) { 
        if ( std::isprint(v)) {
            _log << (char) v;
        }
        else {
            _log << std::hex << v;
        }
    }
    _log << "]" << endl;

    vector<uint8_t> thePayloadHash(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&thePayloadHash[0], reinterpret_cast<const uint8_t*>(inPayload.data()), inPayload.size());

    _log << "\tSigner: " << GetHGID() << " Type: sign_payload hash: [";
    for (int v: thePayloadHash) { _log << std::hex << v; }
    _log << "] ";
    _log << endl;    
 
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::Signature sig = sk.SignPrehashed(thePayloadHash.data());
    return sig;
}

// transmit message
void MeshNode::SendTransmission(const MeshMessage& inMessage)
{
    _log << "Node " << GetHGID() << ", ";
    _log << "<< SendTransmission: " << inMessage << endl << endl;
    double tx_distance = Distance(mCurrentPos, MeshNode::FromHGID(inMessage.mReceiver).mCurrentPos);
    _log << "\tDistance = " << tx_distance << " meters" << endl;
    
    if (!VerifyMessage(inMessage)) {
        // _log << "\tVerifyMessage, failed!" << endl;
        return;
    }
    MeshNode& receiver = MeshNode::FromHGID(inMessage.mReceiver);
    receiver.ReceiveTransmission(inMessage);
}

// receive message
void MeshNode::ReceiveTransmission(const MeshMessage &inMessage)
{
    _log << "Node " << GetHGID() << ", ";
    _log << ">> ReceiveTransmission." << endl;
    MeshMessage theMessage = inMessage;

    // check that aggregate signature is valid
    MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);
    if (!VerifyMessage(inMessage)) {
        _log << "\tVerifyMessage, failed!" << endl;
        return;
    }

    // handle setup1 (fund channel)
    if (theMessage.mIncentive.mType == eSetup1) {
        FundChannel(theMessage);
        return;
    }

    // handle receipt (relay receipt)
    if (theMessage.mIncentive.mType == eReceipt1 || theMessage.mIncentive.mType == eReceipt2) {
        RelayDeliveryReceipt(theMessage);
        return;
    }

    // handle negotiate 1 (relay message)
    if (theMessage.mDestination != GetHGID()) {
        RelayMessage(theMessage);
        return;
    }
    else {
        ReceiveMessage(theMessage);
        return;
    }

    // unhandled message
    assert(false);
}

// check that aggregate signature is valid
bool MeshNode::VerifyMessage(const MeshMessage& inMessage) 
{
    _log << "Node " << GetHGID() << ", ";
    _log << "VerifyMessage: " << inMessage << endl;

    bls::Signature agg_sig = GetAggregateSignature(inMessage, false);
    if (!agg_sig.Verify()) {
        assert(0);
        return false;
    } 
    return true;
}

bool MeshNode::GetNearestGateway(HGID& outGateway)
{
    // node already a gateway?
    if (mIsGateway) {
        outGateway = GetHGID();
        return true;
    }

    double min_distance = std::numeric_limits<double>::max();
    bool route_found = false;
    for (auto& n : sNodes) {
        std::list<HGID> visited;
        double distance;
        MeshRoute route;
        HGID hgid = n.GetHGID();
        bool found = FindRoute(hgid, route, visited, distance);
        if (found && distance < min_distance) {
            outGateway = hgid;
            route_found = true;
        }
    }
    return route_found;
}

// set witness node
void MeshNode::SetWitnessNode(const HGID inHGID)
{
    mWitnessNode = inHGID;
}

// set corresondent node
void MeshNode::SetCorrespondentNode(const HGID inHGID)
{
    mCorrespondent = inHGID;
}

HGID MeshNode::GetCorrespondentNode() const
{
    return mCorrespondent;
}

// is gateway node?
bool MeshNode::GetIsGateway() const
{
    return mIsGateway;
}

// compute serialization of the L49Header for Witness verification
std::vector<uint8_t> L49Header::Serialize() const
{
    uint32_t offset = 0;
    std::vector<uint8_t> buf(4 + mRelayPath.size()*sizeof(HGID) + bls::Signature::SIGNATURE_SIZE);
    buf[offset++] = mWitness ? 0x1 : 0x0;
    buf[offset++] = (uint8_t) mType;
    buf[offset++] = mPrepaidTokens;
    uint8_t relay_path_size = mRelayPath.size();
    assert(relay_path_size < 10);
    buf[offset++] = relay_path_size;
    std::copy((uint8_t*) &mRelayPath[0], (uint8_t*) (&mRelayPath[0]) + relay_path_size*sizeof(HGID), &buf[offset]);
    offset += mRelayPath.size()*sizeof(HGID);
    std::copy(&mSignature[0], &mSignature[0] + bls::Signature::SIGNATURE_SIZE, &buf[offset]);
    offset += bls::Signature::SIGNATURE_SIZE;
    assert(offset == buf.size());

    return buf;
}

// compute serialization of the Mesh Message for Witness verification
std::vector<uint8_t> MeshMessage::Serialize() const
{
    std::vector<uint8_t> incent_buf = mIncentive.Serialize();
    std::vector<uint8_t> buf(4 * sizeof(HGID) + 1 + incent_buf.size() + 2 + mPayloadData.size());

    uint32_t offset = 0;
    std::copy((char*) &mSender, (char*)(&mSender) + sizeof(mSender), &buf[offset]);
    offset += sizeof(mSender);
    std::copy((char*) &mReceiver, (char*)(&mReceiver) + sizeof(mReceiver), &buf[offset]);
    offset += sizeof(mReceiver);
    std::copy((char*) &mSource, (char*)(&mSource) + sizeof(mSource), &buf[offset]);
    offset += sizeof(mSource);
    std::copy((char*) &mDestination, (char*)(&mDestination) + sizeof(mDestination), &buf[offset]);
    offset += sizeof(mDestination);
    uint8_t incent_size = incent_buf.size();
    buf[offset++] = incent_size;
    std::copy(incent_buf.begin(), incent_buf.end(), &buf[offset]);
    offset += incent_size;
    assert(incent_size <= 255);
    uint16_t payload_size = mPayloadData.size();
    assert(payload_size <= std::numeric_limits<uint16_t>::max());
    std::copy((char*) &payload_size, (char*)(&payload_size) + sizeof(payload_size), &buf[offset]);
    offset += sizeof(payload_size);
    std::copy(mPayloadData.begin(), mPayloadData.end(), &buf[offset]);
    offset += payload_size;
    assert(offset == buf.size());

    return buf;
}

// reconstruct MeshMessage from serialized data
void MeshMessage::FromBytes(const std::vector<uint8_t>& inData) 
{
    uint32_t offset = 0;
    mSender = *reinterpret_cast<const HGID*>(&inData[offset]);
    offset += sizeof(mSender);
    mReceiver = *reinterpret_cast<const HGID*>(&inData[offset]);
    offset += sizeof(mReceiver);
    mSource = *reinterpret_cast<const HGID*>(&inData[offset]);
    offset += sizeof(mSource);
    mDestination = *reinterpret_cast<const HGID*>(&inData[offset]);
    offset += sizeof(mDestination);
    uint8_t incent_size = inData[offset++];
    std::vector<uint8_t> incent_buf(incent_size);
    std::copy(&inData[offset], &inData[offset] + incent_size, &incent_buf[0]);
    mIncentive.FromBytes(incent_buf);
    offset += incent_size;
    uint16_t payload_size;
    payload_size = *reinterpret_cast<const uint16_t*>(&inData[offset]);
    offset += sizeof(payload_size);
    mPayloadData.resize(payload_size);
    std::copy(&inData[offset], &inData[offset] + payload_size, &mPayloadData[0]);
    offset += payload_size;
    assert(offset = inData.size());  
}

// reconstruct L49Header from serialized data
void L49Header::FromBytes(const std::vector<uint8_t>& inData) 
{
    uint32_t offset = 0;
    mWitness = (inData[offset++] == 0x1 ? true : false);
    mType = (EChannelState) inData[offset++];
    mPrepaidTokens = inData[offset++];
    uint8_t relay_path_size = inData[offset++];
    mRelayPath.resize(relay_path_size);
    if (relay_path_size > 0) {
        std::copy(&inData[offset], &inData[offset] + relay_path_size * sizeof(HGID), reinterpret_cast<uint8_t*>(&mRelayPath[0]));
        offset += relay_path_size*sizeof(HGID);
    }
    mSignature.resize(bls::Signature::SIGNATURE_SIZE);
    std::copy(&inData[offset], &inData[offset] + bls::Signature::SIGNATURE_SIZE, &mSignature[0]);
    offset += bls::Signature::SIGNATURE_SIZE;
    assert(offset == inData.size());
}

static std::ofstream sTopology;
std::ofstream& TOPOLOGY() {
    if (!sTopology.is_open()) {
        sTopology.open("lot49_topology.csv");
        sTopology << "time, node, correspondent, distance, current x, current y, paused" << endl;
    }
    return sTopology;
};
#define _topology TOPOLOGY()

void MeshNode::PrintTopology() 
{
    // header
    _log << "     \t";
    for (auto& node1 : sNodes) {
        _topology << sCurrentTime << ", ";
        _topology << std::hex << std::setw(4) << node1.GetHGID() << ", ";
        _topology << std::hex << std::setw(4) << node1.mCorrespondent << ", ";
        _topology << std::dec << (int) Distance(node1.mCurrentPos, MeshNode::FromHGID(node1.mCorrespondent).mCurrentPos) << ", ";
        _topology << std::dec << (int) node1.mCurrentPos.first << ", ";
        _topology << std::dec << (int) node1.mCurrentPos.second << ", ";
        _topology << (node1.mPausedUntil > sCurrentTime ? "true" : "false") << endl;
    }
    
    /*
    for (auto& node1 : sNodes) {
        _log << std::hex << std::setw(4) << node1.GetHGID() << "   ";
        for (auto& node2 : sNodes) {
            if (node1.GetHGID() == node2.GetHGID()) {
                _log << "\t ---- ";
            }
            else {
                MeshRoute route;
                std::list<HGID> visited;
                double distance;
                bool found = node1.FindRoute(node2.GetHGID(), route, visited, distance);
                if (found) {
                    _log << "\t" << (float)distance << "(" << route.size() << ") ";
                }
                else {
                    _log << "\t XXXX ";
                }
            }
        }
        _log << endl;
    }
    */
}

// update position of all nodes, update routes and send messages
void MeshNode::UpdateSimulation()
{
    // update position of all nodes
    for (auto& n : sNodes) {
        // paused at waypoint
        if (n.mPausedUntil > sCurrentTime) {
            continue;
        }
        // if at waypoint, pick new waypoint and pause
        double distance = Distance(n.mCurrentPos, n.mWaypoint);
        if (distance <= sMoveRate) {
            std::uniform_int_distribution<double> pos(-sMaxSize/2, sMaxSize/2);
            n.mWaypoint.first = pos(rng);
            n.mWaypoint.second = pos(rng);
            n.mPausedUntil = sCurrentTime + sPauseTime;
            distance = Distance(n.mCurrentPos, n.mWaypoint);
        }
        double dX = ((n.mWaypoint.first - n.mCurrentPos.first)/distance) * sMoveRate; // in meters per minute
        double dY = ((n.mWaypoint.second - n.mCurrentPos.second)/distance) * sMoveRate;

        // update position (wrap around instead of moving out of area)
        n.mCurrentPos.first = -sMaxSize/2 + ((int)(sMaxSize/2 + n.mCurrentPos.first + dX) % (int) sMaxSize);
        n.mCurrentPos.second = -sMaxSize/2 + ((int)(sMaxSize/2 + n.mCurrentPos.second + dY) % (int) sMaxSize);
    }

    // update links to nearby nodes
    ClearRoutes();
    for (auto& n1 : sNodes) {
        for (MeshNode& n2 : sNodes) {

            // skip nodes out of range and skip self
            double distance = Distance(n1.mCurrentPos, n2.mCurrentPos);
            if (distance > sRadioRange || n1.GetHGID() == n2.GetHGID()) {
                continue;
            }

            // propose channel to any nearby node
            if (!MeshNode::HasNeighbor(n1.GetHGID(),n2.GetHGID())) {
                MeshRoute route = {n1.GetHGID(), n2.GetHGID()};
                n1.AddRoute(route);
            }
        }
    }

    // log current relative positions of nodes
    PrintTopology();

    // send message to correspondent each round
    for (auto& n : sNodes) {
        if (n.GetHGID() != n.mCorrespondent) {
            // send a message and receive delivery receipt
            std::vector<uint8_t> payload(sPayloadSize, 'A');
            if (n.OriginateMessage(n.mCorrespondent, payload)) {
                cout << std::hex << endl << "Node " << std::setw(4) << std::setfill('0') << n.GetHGID() << ": send a message to Node " << std::setw(4) << n.mCorrespondent << endl;
            }
            else {
                cout << std::hex << endl << "Node " << std::setw(4) << std::setfill('0') << n.GetHGID() << ": no route to Node " << std::setw(4) << n.mCorrespondent << endl;
            }
        }
    }

    sCurrentTime++;
}


}; // namespace lot49