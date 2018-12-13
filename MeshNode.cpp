
#include "MeshNode.hpp"
#include <random>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <fstream>

using namespace std;
using namespace lot49;

static std::ofstream sLogfile;
std::ofstream& LOG() {
    if (!sLogfile.is_open()) {
        sLogfile.open("lot49.log");
    }
    return sLogfile;
};
#define _log LOG()

std::vector<lot49::MeshNode> lot49::MeshNode::sNodes;
std::list<lot49::MeshRoute> lot49::MeshNode::sRoutes;
std::list<lot49::HGID> lot49::MeshNode::sGateways;

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
    return out;
}

std::ostream &operator<<(std::ostream &out, const MeshNode &n)
{
    out << "HGID: " << std::hex << std::setw(4) << std::setfill('0') << n.GetHGID() << endl;
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
   return out;
}

std::ostream &operator<<(std::ostream &out, const MeshMessage &m)
{
    out << "Message, Source: " << std::setw(2) << std::setfill('0') << m.mSource;
    out << " Sender: " << std::setw(2) << std::setfill('0') << m.mSender;
    out << " Receiver: " << std::setw(2) << std::setfill('0') << m.mReceiver;
    out << " Destination: " << std::setw(2) << std::setfill('0') << m.mDestination;
    if (m.mPayloadData.empty()) {
        out << " Payload: []" << endl;
    }
    else if (m.mIncentive.mWitness) {
        MeshMessage witness_message;
        witness_message.FromBytes(m.mPayloadData);
        out << "\tPayload: Witness " << witness_message << endl;
    }
    else {
        std::string payload_text(reinterpret_cast<const char*>(m.mPayloadData.data()), m.mPayloadData.size());
        out << " Payload: [" << payload_text << "]" << endl;
    }
    out << "\t" << m.mIncentive;
    return out;
}

// create mesh nodes
void MeshNode::CreateNodes(const int inCount)
{

    sNodes.resize(inCount);
    //std::generate(sNodes.begin(), sNodes.end(), [&] {return MeshNode();});

    for (auto n: sNodes) {
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

HGID MeshNode::GetNextHop(HGID inNode, HGID inDestination)
{
    for (auto route = sRoutes.begin(); route != sRoutes.end(); ++route) {
        auto node_iter = std::find(route->begin(), route->end(), inNode);
        auto neighbor_iter = std::find(route->begin(), route->end(), inDestination);
        if (node_iter != route->end() && neighbor_iter != route->end()) {
            std::ptrdiff_t diff = std::distance(node_iter, neighbor_iter);
            assert(diff != 0);
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
        _log << "Node " << std::hex << *hgid_iter << ", Neighbor " << *(hgid_iter+1) << endl;
        FromHGID(*hgid_iter).ProposeChannel(*(hgid_iter+1));
    }

    // propose channels with neighbors (backwards)
    auto route_rend = inRoute.rbegin() + (inRoute.size() - 1);
    for (auto hgid_iter = inRoute.rbegin(); hgid_iter != route_rend; ++hgid_iter) {
        _log << "Node: " << std::hex << *hgid_iter << " Neighbor: " << *(hgid_iter+1) << endl;
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
    
    // if no witness set, then use same hgid as node
    mWitnessNode = GetHGID();
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

    _log << "Node " << GetHGID() << ", ";
    _log << "ProposeChannel to " << inNeighbor << endl;

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
    bls::Signature refund_sig = SignTransaction(theImpliedTransactions.front());   

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
void MeshNode::OriginateMessage(const HGID inDestination, const std::vector<uint8_t>& inPayload)
{
    std::string payload_text(reinterpret_cast<const char*>(inPayload.data()), inPayload.size());
    _log << "Node " << GetHGID() << ", ";
    _log << "OriginateMessage, Destination: " << inDestination << ", Payload: [" << payload_text << "]" << endl << endl;

    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = GetNextHop(GetHGID(), inDestination);
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inDestination;
    theMessage.mPayloadData = inPayload;

    PeerChannel &theChannel = GetChannel(theMessage.mReceiver);

    assert(theChannel.mState == eSetup2 || theChannel.mState == eReceipt2 || theChannel.mState == eReceipt1);
    assert(theChannel.mUnspentTokens >= PREPAID_TOKENS);

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
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], theMessage.mPayloadData.data(), theMessage.mPayloadData.size()); 
    _log << "\tSave payload hash: [";
    for (int v: theChannel.mPayloadHash) { _log << std::hex << v; }
    _log << "] ";
    _log << endl;    

    UpdateIncentiveHeader(theMessage);

    SendTransmission(theMessage);
}

// relay a message
void MeshNode::RelayMessage(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "RelayMessage: " << inMessage << endl;

    // confirm setup transaction on the blockchain
    PeerChannel &theSenderChannel = GetChannel(inMessage.mSender);
    if (theSenderChannel.mConfirmed == false) {
        ConfirmSetupTransaction(inMessage, GetNearestGateway(GetHGID()));
    }

    // pay next hop
    uint8_t prepaid_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel &theChannel = GetChannel(GetNextHop(GetHGID(), inMessage.mDestination));
    theChannel.mUnspentTokens -= prepaid_tokens;
    theChannel.mSpentTokens += prepaid_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = inMessage.mIncentive.mType;
    
    // save a local copy of the payload hash for confirming receipt2 messages
    assert ( inMessage.mIncentive.mType < eReceipt1 );
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());
    _log << "\tSave payload hash: [";
    for (int v: theChannel.mPayloadHash) { _log << std::hex << v; }
    _log << "] ";
    _log << endl;

    // new relay message
    MeshMessage outMessage = inMessage;
    outMessage.mSender = GetHGID();
    outMessage.mReceiver = GetNextHop(GetHGID(), inMessage.mDestination);

    if (outMessage.mReceiver == outMessage.mDestination && outMessage.mIncentive.mType == eNegotiate1 ) {
        // next node is destination node
        outMessage.mIncentive.mType = eNegotiate2;
        theChannel.mState = eNegotiate2;
    }

    UpdateIncentiveHeader(outMessage);

    // send message to next hop
    SendTransmission(outMessage);
}

// fund a channel
void MeshNode::FundChannel(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "FundChannel: " << inMessage << endl;
    
    // TODO: check that funds exist, etc.

    // create channel entry for peer that proposed the channel
    PeerChannel theChannel;
    theChannel.mFundingPeer = HGID();
    theChannel.mProposingPeer = inMessage.mSender;
    theChannel.mUnspentTokens = inMessage.mIncentive.mPrepaidTokens;
    theChannel.mSpentTokens = 0;
    theChannel.mLastNonce = 0;
    theChannel.mState = eSetup2;
    theChannel.mRefundSignature = inMessage.mIncentive.mSignature;

    // save a local copy of the payload hash for confirming receipt2 messages
    assert ( inMessage.mIncentive.mType < eReceipt1 );
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());
    
    mPeerChannels.push_back(theChannel);
}

// receive message
void MeshNode::ReceiveMessage(const MeshMessage& inMessage)
{    
    _log << "Node " << GetHGID() << ", ";
    _log << "ReceiveMessage: " << inMessage << endl;

    // receive payment
    uint8_t remaining_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
    theChannel.mUnspentTokens -= remaining_tokens;
    theChannel.mSpentTokens += remaining_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = eReceipt1;
 
    // save a local copy of the payload hash for confirming receipt2 messages
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

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
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
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

        // send proof of receipt to previous hop
        SendTransmission(theMessage);
    }
    else if (inMessage.mIncentive.mWitness) {
        _log << "Confirmation of channel setup received by " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from Witness Node " << std::setw(4) << inMessage.mDestination << "!" << endl;
        cout << "Confirmation of channel setup received by " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from Witness Node " << std::setw(4) << inMessage.mDestination << "!" << endl;
    }
    else {
        _log << "Delivery Receipt received by " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from message destination " << std::setw(4) << inMessage.mDestination << "!" << endl;
        cout << "Delivery Receipt received by " << std::setw(4) << std::setfill('0') << inMessage.mSource << " from message destination " << std::setw(4) << inMessage.mDestination << "!" << endl;
    }
}

// confirm the setup transaction for a payment channel with a witness node (via inGateway) 
void MeshNode::ConfirmSetupTransaction(const MeshMessage& inMessage, const HGID inGateway)
{
    _log << "Node " << GetHGID() << ", ";
    _log << "ConfirmSetupTransaction, Gateway: " << inGateway << ", Message Hash: [" << inMessage << "]" << endl << endl;

    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = GetNextHop(GetHGID(), inGateway);
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inGateway;

    theMessage.mPayloadData = inMessage.Serialize();

    PeerChannel &theChannel = GetChannel(theMessage.mReceiver);

    assert(theChannel.mState == eSetup2 || theChannel.mState == eReceipt2);
    assert(theChannel.mUnspentTokens >= PREPAID_TOKENS);

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
    theChannel.mPayloadHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    std::vector<uint8_t> theMeshMessage = inMessage.Serialize();
    bls::Util::Hash256(&theChannel.mPayloadHash[0], reinterpret_cast<const uint8_t*>(theMessage.mPayloadData.data()), theMessage.mPayloadData.size()); 

    UpdateIncentiveHeader(theMessage);
    SendTransmission(theMessage);
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

        HGID next_hop_hgid = (GetHGID() == inMessage.mDestination ? GetNextHop(GetHGID(), inMessage.mSource) : GetNextHop(GetHGID(), inMessage.mDestination));
        const PeerChannel& theChannel = GetChannel( next_hop_hgid );

        _log << endl << "\tSigner: " << inMessage.mDestination << " Type: sign_payload* hash: [";
        for (int v: theChannel.mPayloadHash) { _log << std::hex << v; }
        _log << "] ";
        _log << endl;

        aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(pk, theChannel.mPayloadHash.data()) );
    }    

    _log << endl;

    bls::AggregationInfo merged_aggregation_info = bls::AggregationInfo::MergeInfos(aggregation_info);
    bls::Signature agg_sig = bls::Signature::FromBytes(inMessage.mIncentive.mSignature.data());
    agg_sig.SetAggregationInfo(merged_aggregation_info);
    sigs.push_back(agg_sig);

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

    // _log << "GetTransactions, Type: " << incentive.mType << endl;

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
            
            //_log << endl << "\t\t public key: " << sender.GetPublicKey() << ", tokens: " << (int) prepaid_tokens << ", sender:" << sender.GetHGID() << ", receiver:" << receiver.GetHGID() << " tx: [";
            //for (int v: theTransactions.back().GetTransactionHash()) { _log << std::hex << v; }
            //_log << "] " << endl;

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
    std::string theTextPayload(reinterpret_cast<const char*>(inPayload.data()), inPayload.size());
    _log << "Node " << GetHGID() << ", ";
    _log << "SignMessage: " << theTextPayload << endl;

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

void MeshNode::AddGateway(HGID inNode)
{
    sGateways.push_back(inNode);
}

HGID MeshNode::GetNearestGateway(HGID inFromNode)
{
    // TODO: use location to find nearest gateway
    if (!sGateways.empty()) {
        return sGateways.front();
    }
    return inFromNode;
}

// set witness node
void MeshNode::SetWitnessNode(const HGID inWitness)
{
    mWitnessNode = inWitness;
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
    std::vector<uint8_t> buf(5 * sizeof(HGID) + incent_buf.size() + 1 + mPayloadData.size());

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
    uint8_t payload_size = mPayloadData.size();
    buf[offset++] = payload_size;
    std::copy(mPayloadData.begin(), mPayloadData.end(), &buf[offset]);
    offset += payload_size;
    assert(offset = buf.size());

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
    uint8_t payload_size = inData[offset++];
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


}; // namespace lot49