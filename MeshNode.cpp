
#include "MeshNode.hpp"
#include <random>
#include <algorithm>
#include <cassert>
#include <iterator>

using namespace std;
using namespace lot49;

std::vector<lot49::MeshNode> lot49::MeshNode::sNodes;
std::list<lot49::MeshRoute> lot49::MeshNode::sRoutes;

static std::default_random_engine rng(std::random_device{}());
static std::uniform_int_distribution<uint8_t> dist(0, 255); //(min, max)

//get one
const double random_num = dist(rng);

namespace lot49
{

static uint8_t COMMITTED_TOKENS = 100;
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
}

std::ostream &operator<<(std::ostream &out, const MeshNode &n)
{
    out << "HGID: " << std::hex << std::setw(4) << std::setfill('0') << n.GetHGID() << endl;
    out << "\tPublic Key: " << n.GetPublicKey() << endl;
    out << "\tSeed: ";
    out << std::hex;
    for (auto  byte : n.mSeed ) {
        out << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    } 
}

std::ostream& operator<<(std::ostream &out, const L49Header &i)
{
    out << "Incentive, Type: " << i.mType << " Prepaid Tokens: " << (int) i.mPrepaidTokens << endl;
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
}

std::ostream &operator<<(std::ostream &out, const MeshMessage &m)
{
    out << "Message, Source: " << std::setw(2) << std::setfill('0') << m.mSource;
    out << " Sender: " << std::setw(2) << std::setfill('0') << m.mSender;
    out << " Receiver: " << std::setw(2) << std::setfill('0') << m.mReceiver;
    out << " Destination: " << std::setw(2) << std::setfill('0') << m.mDestination << endl;
    out << "\t" << m.mIncentive;
}

// create mesh nodes
void MeshNode::CreateNodes(const int inCount)
{

    sNodes.resize(inCount, MeshNode());
    std::generate(sNodes.begin(), sNodes.end(), [&] {return MeshNode();});
}

MeshNode &MeshNode::FromIndex(const int inIndex)
{
    if (inIndex > sNodes.size())
    {
        CreateNodes(inIndex + 1);
    }
    return sNodes[inIndex];
}

//  Lookup or construct a node from a Hashed GID
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

HGID MeshNode::GetNextHop(HGID inNode, HGID inDestination)
{
    for (auto route = sRoutes.begin(); route != sRoutes.end(); ++route) {
        auto node_iter = std::find(route->begin(), route->end(), inNode);
        auto neighbor_iter = std::find(route->begin(), route->end(), inDestination);
        if (node_iter != route->end() && neighbor_iter != route->end()) {
            std::ptrdiff_t diff = std::distance(node_iter, neighbor_iter);
            MeshRoute::iterator next_hop_iter = node_iter + (diff/abs(diff));
            return *next_hop_iter;
        }
    }
    throw std::invalid_argument("No route to destination.");
}

// configure topology
void MeshNode::AddRoute(MeshRoute inRoute)
{
    sRoutes.push_back(inRoute);

    // propose channels with neighbors (forward)
    auto route_end = inRoute.begin() + (inRoute.size() - 1);
    for (auto hgid_iter = inRoute.begin(); hgid_iter != route_end; ++hgid_iter) {
        cout << "Node: " << std::hex << *hgid_iter << " Neighbor: " << *(hgid_iter+1) << endl;
        FromHGID(*hgid_iter).ProposeChannel(*(hgid_iter+1));
    }

    // propose channels with neighbors (backwards)
    auto route_rend = inRoute.rbegin() + (inRoute.size() - 1);
    for (auto hgid_iter = inRoute.rbegin(); hgid_iter != route_rend; ++hgid_iter) {
        cout << "Node: " << std::hex << *hgid_iter << " Neighbor: " << *(hgid_iter+1) << endl;
        FromHGID(*hgid_iter).ProposeChannel(*(hgid_iter+1));
    }
}

bool MeshNode::HasNeighbor(HGID inNode, HGID inNeighbor)
{
    bool found = false;
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

    cout << "Node " << GetHGID() << ", ";
    cout << "ProposeChannel to " << inNeighbor << endl;

    PeerChannel theChannel;
    theChannel.mFundingPeer = inNeighbor;
    theChannel.mProposingPeer = HGID();
    theChannel.mUnspentTokens = COMMITTED_TOKENS;
    theChannel.mSpentTokens = 0;
    theChannel.mLastNonce = 0;
    theChannel.mState = eSetup1;
    theChannel.mConfirmed = false;
    
    mPeerChannels.push_back(theChannel);

    MeshMessage theMessage;
    theMessage.mSource = inNeighbor;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inNeighbor;
    theMessage.mDestination = inNeighbor;
        
    // initialize incentive aggregate signature by signing refund tx
    theMessage.mIncentive.mType = theChannel.mState;
    theMessage.mIncentive.mPrepaidTokens = theChannel.mUnspentTokens;
    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(theMessage);
    bls::Signature refund_sig = SignTransaction(theImpliedTransactions.back());    
    theMessage.mIncentive.mSignature.resize(bls::Signature::SIGNATURE_SIZE);
    refund_sig.Serialize(&theMessage.mIncentive.mSignature[0]);

    SendTransmission(theMessage);
}

// return true if channel exists with this neighbor
bool MeshNode::HasChannel(HGID inNeighbor) const
{
    for (auto channel = mPeerChannels.begin(); channel != mPeerChannels.end(); ++channel) {
        if (channel->mProposingPeer == inNeighbor) {
            return true;
        }
    }
    return false;
}

// get existing channel
PeerChannel& MeshNode::GetChannel(HGID inNeighbor)
{
    for (auto channel = mPeerChannels.begin(); channel != mPeerChannels.end(); ++channel) {
        if (channel->mProposingPeer == inNeighbor) {
            return *channel;
        }
    }
    throw std::invalid_argument("No channel exists for neighbor.");
}

// originate new message
void MeshNode::OriginateMessage(const HGID inDestination, const std::string &inPayload)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "OriginateMessage, Destination: " << inDestination << ", Payload: [" << inPayload << "]" << endl << endl;

    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = GetNextHop(GetHGID(), inDestination);
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inDestination;
    theMessage.mPayloadData = inPayload;

    PeerChannel &theChannel = GetChannel(theMessage.mReceiver);

    assert(theChannel.mState == eSetup2 || theChannel.mState == eReceipt2);
    assert(theChannel.mUnspentTokens >= PREPAID_TOKENS);

    theChannel.mUnspentTokens -= PREPAID_TOKENS;
    theChannel.mSpentTokens += PREPAID_TOKENS;
    theChannel.mLastNonce += 1;
    theChannel.mState = eNegotiate1;
    
    theMessage.mIncentive.mPrepaidTokens = PREPAID_TOKENS;
    theMessage.mIncentive.mSignature = theChannel.mRefundSignature;
    theMessage.mIncentive.mType = eNegotiate1;

    // save a local copy of the payload hash for confirming receipt2 messages
    assert(theMessage.mIncentive.mType < eReceipt1 );
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(theMessage.mPayloadData.data()), theMessage.mPayloadData.size()); 

    UpdateIncentiveHeader(theMessage);

    SendTransmission(theMessage);
}

// relay a message
bool MeshNode::RelayMessage(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "RelayMessage: " << inMessage << endl;

    // receive payment
    uint8_t prepaid_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel &theChannel = GetChannel(GetNextHop(GetHGID(), inMessage.mDestination));
    theChannel.mUnspentTokens -= prepaid_tokens;
    theChannel.mSpentTokens += prepaid_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = inMessage.mIncentive.mType;
    theChannel.mConfirmed = false;
    
    // save a local copy of the payload hash for confirming receipt2 messages
    assert ( inMessage.mIncentive.mType < eReceipt1 );
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

    // new relay message
    MeshMessage outMessage = inMessage;
    outMessage.mSender = GetHGID();
    outMessage.mReceiver = GetNextHop(GetHGID(), inMessage.mDestination);

    if (outMessage.mReceiver == outMessage.mDestination && outMessage.mIncentive.mType == eNegotiate1 ) {
        // next node is destination node
        outMessage.mIncentive.mType = eNegotiate2;
    }
    theChannel.mState = outMessage.mIncentive.mType;

    UpdateIncentiveHeader(outMessage);

    // send payment
    {
        uint8_t commited_tokens = (outMessage.mIncentive.mPrepaidTokens - (outMessage.mIncentive.mRelayPath.size() + 1));
        PeerChannel &theChannel = GetChannel(outMessage.mReceiver);
        theChannel.mUnspentTokens += commited_tokens;
        theChannel.mSpentTokens -= commited_tokens;
        theChannel.mLastNonce += 1;
        theChannel.mState = outMessage.mIncentive.mType;
        theChannel.mConfirmed = false;
    }

    // send message to next hop
    SendTransmission(outMessage);

    return true;
}

// fund a channel
bool MeshNode::FundChannel(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "FundChannel: " << inMessage << endl;
    
    // TODO: check that funds exist, etc.

    // create channel entry for peer that proposed the channel
    PeerChannel theChannel;
    theChannel.mFundingPeer = HGID();
    theChannel.mProposingPeer = inMessage.mSender;
    theChannel.mUnspentTokens = inMessage.mIncentive.mPrepaidTokens;
    theChannel.mSpentTokens = 0;
    theChannel.mLastNonce = 0;
    theChannel.mState = eSetup2;
    theChannel.mConfirmed = false;
    theChannel.mRefundSignature = inMessage.mIncentive.mSignature;

    // save a local copy of the payload hash for confirming receipt2 messages
    assert ( inMessage.mIncentive.mType < eReceipt1 );
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN,1);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());
    
    mPeerChannels.push_back(theChannel);

    return true;
}

// receive message
bool MeshNode::ReceiveMessage(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "ReceiveMessage: " << inMessage << endl;

    // receive payment
    uint8_t remaining_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
    theChannel.mUnspentTokens -= remaining_tokens;
    theChannel.mSpentTokens += remaining_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = eReceipt1;
    theChannel.mConfirmed = false;
 
    // save a local copy of the payload hash for confirming receipt2 messages
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN,3);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());
     
    // send return receipt
    MeshMessage theMessage = inMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inMessage.mSender;
    theMessage.mIncentive.mType = eReceipt1;
    UpdateIncentiveHeader(theMessage);

    // send proof of receipt to previous hop
    SendTransmission(theMessage);

    return true;
}

// receive delivery receipt
bool MeshNode::RelayDeliveryReceipt(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "RelayDeliveryReceipt: " << inMessage << endl;

    // destination node confirms message hash matches
    if (!VerifyMessage(inMessage)) {
        cout << "\tVerifyMessage, failed!" << endl;
        return false;
    }

    // TODO: process confirmed payment
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
    theChannel.mState = eReceipt2;

    if (GetHGID() != inMessage.mSource) {
        // relay return receipt
        MeshMessage theMessage = inMessage;
        theMessage.mSender = GetHGID();
        theMessage.mReceiver = GetNextHop(GetHGID(), inMessage.mSource);

        // message destination signs before relaying eReceipt1, all others just relay
        assert(theMessage.mIncentive.mType == eReceipt1 || theMessage.mIncentive.mType == eReceipt2);
        theMessage.mIncentive.mType == eReceipt2;
        // no need to call UpdateIncentiveHeader(theMessage) because receipts don't need any extra incentives

        // send proof of receipt to previous hop
        SendTransmission(theMessage);
    }
    else {
        cout << "Delivery Receipt received by " << inMessage.mSource << " from message destination " << inMessage.mDestination << "!" << endl << endl;
    }

    return true;
}

bls::Signature MeshNode::GetAggregateSignature(const MeshMessage& inMessage, const bool isUpdatingSignature)
{
    // cout << "Node " << GetHGID() << ", ";
    // cout << "GetAggregateSignature, " << inMessage << endl;

    // calculate aggregation info from implied transaction hashes and signing public keys  
    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(inMessage);
    vector<bls::Signature> sigs;
    std::vector<bls::AggregationInfo> aggregation_info;
    bool isPreviousTx = !isUpdatingSignature;
    for (auto tx = theImpliedTransactions.rbegin(); tx != theImpliedTransactions.rend(); tx++) {
        bls::PublicKey pk = bls::PublicKey::FromBytes(tx->mTransactionSigner.data());

        // only add aggregation_info for previously aggregated signatures
        isPreviousTx |= !(pk == GetPublicKey());
        if (isPreviousTx) {
            aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(pk, tx->GetTransactionHash().data()) );
        }
        else {
            // push current signature contained in the message
            sigs.push_back( SignTransaction(*tx) );
        }
        /*
        cout << "\tType: " << tx->GetType() << (isPreviousTx ? "* " : " ") << " tx: [";
        for (int v: tx->GetTransactionHash()) { std::cout << std::hex << v; }
        cout << "] ";
        cout << "\tSigner PK: ";
        for (int v : tx->mTransactionSigner) { cout << std::hex << v; }
        cout << endl;
        */
    }

    // add aggregation info for destination's signature for payload message
    if (inMessage.mIncentive.mType >= eReceipt2 || (inMessage.mIncentive.mType == eReceipt1 && !isUpdatingSignature)) {
        const MeshNode& destination = MeshNode::FromHGID(inMessage.mDestination);
        bls::PublicKey pk = destination.GetPublicKey();

        HGID next_hop_hgid = GetHGID() == inMessage.mDestination ? GetNextHop(GetHGID(), inMessage.mSource) : GetNextHop(GetHGID(), inMessage.mDestination);
        const PeerChannel& theChannel = GetChannel( next_hop_hgid );

        /*
        cout << "\tnext_hop_hgid = "<< std::hex << (int)next_hop_hgid << endl;
        cout << "\tType: sign_payload* hash: [";
        for (int v: theChannel.mPayloadHash) { std::cout << std::hex << v; }
        cout << "] ";
        cout << "\tSigner PK: ";
        for (int v : pk.Serialize()) { cout << std::hex << v; }
        cout << endl;
        */

        aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(pk, theChannel.mPayloadHash.data()) );
    }    

    //cout << endl;

    bls::AggregationInfo merged_aggregation_info = bls::AggregationInfo::MergeInfos(aggregation_info);
    bls::Signature agg_sig = bls::Signature::FromBytes(inMessage.mIncentive.mSignature.data());
    agg_sig.SetAggregationInfo(merged_aggregation_info);
    sigs.push_back(agg_sig);

    if (inMessage.mIncentive.mType == eReceipt1 && isUpdatingSignature) {
        assert(inMessage.mSender == inMessage.mDestination);
        sigs.push_back( SignMessage(inMessage.mPayloadData) );
    }

    // update aggregate signature
    try {
        agg_sig = bls::Signature::AggregateSigs(sigs);
    }
    catch (std::string &e) {
        cout << e << endl;
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
    vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN,4);
    const MeshNode& source = MeshNode::FromHGID(inMessage.mSource);
    const MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);

    HGID first_relay_hgid;
    if (incentive.mType == eSetup1) {
        first_relay_hgid = inMessage.mSender;
    }
    else if (incentive.mRelayPath.empty()) {
        first_relay_hgid = inMessage.mReceiver;
    }
    else {
        first_relay_hgid = incentive.mRelayPath.front();
    }
    const MeshNode& first_relay = MeshNode::FromHGID(first_relay_hgid);

    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

    ImpliedTransaction issued_value_tx = ImpliedTransaction::Issue(source.GetPublicKey(), 255);
    ImpliedTransaction setup_tx = ImpliedTransaction::Setup(issued_value_tx, source.GetPublicKey(), first_relay.GetPublicKey(), COMMITTED_TOKENS);
    ImpliedTransaction refund_tx = ImpliedTransaction::Refund(setup_tx, source.GetPublicKey(), first_relay.GetPublicKey(), COMMITTED_TOKENS);

    // cout << "GetTransactions, Type: " << incentive.mType << endl;

    if (incentive.mType >= eSetup1) {
        // create refund tx
        theTransactions.push_back( refund_tx );
    }
    if (incentive.mType >= eSetup2) {
        // create funding tx
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
            // create update and settle tx
            theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(last_update_tx, sender.GetPublicKey(), receiver.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
            
            //cout << endl << "\t\t public key: " << sender.GetPublicKey() << ", tokens: " << (int) prepaid_tokens << ", sender:" << sender.GetHGID() << ", receiver:" << receiver.GetHGID() << " tx: [";
            //for (int v: theTransactions.back().GetTransactionHash()) { std::cout << std::hex << v; }
            //cout << "] " << endl;

            hgid1 = *hgid2;
            prepaid_tokens--;
            last_update_tx = theTransactions.back();
        }
    }
    if (incentive.mType >= eNegotiate2) {
        // last incentive is from penultimate relay to destination
        const MeshNode& sender = MeshNode::FromHGID(incentive.mRelayPath.back());
        const MeshNode& receiver = MeshNode::FromHGID(inMessage.mDestination);
        // create update and settle tx for delivery to destination node
        theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(theTransactions.back(), sender.GetPublicKey(), receiver.GetPublicKey(), prepaid_tokens, prepaid_tokens-1, destination.GetPublicKey(), message_hash));
    }
    return theTransactions;
}

void MeshNode::UpdateIncentiveHeader(MeshMessage& ioMessage)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "UpdateIncentiveHeader " << ioMessage << endl;


    L49Header &incentive = ioMessage.mIncentive;
    const PeerChannel &theChannel = GetChannel(ioMessage.mReceiver);
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
    //cout << "Node " << GetHGID() << ", ";
    //cout << "SignTransaction, Tx Type: " << inTransaction.GetType() << " tx: [";
    //for (int v: inTransaction.GetTransactionHash()) { std::cout << std::hex << v; } 
    //cout << "]" << endl;
    //cout << "\tTx Signer PK: ";
    //for (int v : inTransaction.mTransactionSigner) { cout << std::hex << v; }
    //cout << endl;
 
    std::vector<uint8_t> msg = inTransaction.Serialize();
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::PublicKey pk = sk.GetPublicKey();
    bls::Signature sig = sk.Sign(msg.data(), msg.size());

    //cout << "\tSignature: " << sig << endl;
    return sig;
}

bls::Signature MeshNode::SignMessage(const std::string& inPayload) const
{
    cout << "Node " << GetHGID() << ", ";
    cout << "SignMessage: " << inPayload << endl;

    vector<uint8_t> thePayloadHash(bls::BLS::MESSAGE_HASH_LEN,5);
    bls::Util::Hash256(&thePayloadHash[0], reinterpret_cast<const uint8_t*>(inPayload.data()), inPayload.size());

    /*
    cout << "\tType: sign_payload hash: [";
    for (int v: thePayloadHash) { std::cout << std::hex << v; }
    cout << "] ";
    cout << "\tSigner PK: ";
    for (int v : GetPublicKey().Serialize()) { cout << std::hex << v; }
    cout << endl;    
    */
 
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::Signature sig = sk.SignPrehashed(thePayloadHash.data());
    return sig;
}

// transmit message
void MeshNode::SendTransmission(const MeshMessage& inMessage)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "<< SendTransmission: >> " << inMessage << endl << endl;
    
    if (!VerifyMessage(inMessage)) {
        // cout << "\tVerifyMessage, failed!" << endl;
        return;
    }
    MeshNode& receiver = MeshNode::FromHGID(inMessage.mReceiver);
    receiver.ReceiveTransmission(inMessage);
}

// receive message
void MeshNode::ReceiveTransmission(const MeshMessage &inMessage)
{
    cout << "Node " << GetHGID() << ", ";
    cout << ">> ReceiveTransmission." << endl;
    MeshMessage theMessage = inMessage;

    // check that aggregate signature is valid
    MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);
    if (!sender.VerifyMessage(inMessage)) {
        cout << "\tVerifyMessage, failed!" << endl;
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
    // cout << "Node " << GetHGID() << ", ";
    // cout << "VerifyMessage: " << inMessage << endl;

    bls::Signature agg_sig = GetAggregateSignature(inMessage, false);
    if (!agg_sig.Verify()) {
        return false;
    } 
    return true;
}

}; // namespace lot49