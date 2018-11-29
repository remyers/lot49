
#include "MeshNode.hpp"
#include <random>
#include <algorithm>
#include <cassert>

using namespace std;
using namespace lot49;

std::vector<lot49::MeshNode> lot49::MeshNode::sNodes;

static std::default_random_engine rng(std::random_device{}());
static std::uniform_int_distribution<uint8_t> dist(0, 255); //(min, max)

//get one
const double random_num = dist(rng);

namespace lot49
{

std::ostream& operator<<(std::ostream &out, const L49Header &i)
{
    out << "Incentive, Type: " << i.mType << " Prepaid Tokens: " << (int) i.mPrepaidTokens << " Relay Path: [";
    for (int v : i.mRelayPath) { out << std::hex << v << ", "; }
    out << "]" << endl;
    out << "\tSignature: ";
    for (int v : i.mSignature) { out << std::hex << v; }
    out << endl;
}

std::ostream &operator<<(std::ostream &out, const MeshMessage &m)
{
    out << "Message, Source: " << m.mSource << " Sender: " << m.mSender << " Receiver: " << m.mReceiver << " Destination: " << m.mDestination << endl;
    out << "\t" << m.mIncentive;
}

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
        if (node->GetHGID() == inHGID)
        {
            return *node;
        }
    }
    throw std::invalid_argument("invalid HGID");
}

// ctor
MeshNode::MeshNode()
{
    mSeed.resize(32);
    std::generate_n(mSeed.begin(), 32, [&] { return dist(rng); });
    cout << "Seed: ";
    for (int v : mSeed) { cout << std::hex << v; }
    cout << endl;
}

HGID MeshNode::GetHGID() const
{
    // TODO: use hash of public key, not first two seed values
    return *reinterpret_cast<const uint16_t *>(mSeed.data());
}

HGID MeshNode::GetNextHop(HGID inDestination) const
{
   for (auto route = mRoutes.begin(); route != mRoutes.end(); ++route)
    {
        if (route->back() == inDestination) {
            return route->front();
        }
    }
    throw std::invalid_argument("No route to destination.");
}

// access public key
const bls::PublicKey MeshNode::GetPublicKey() const
{
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    return sk.GetPublicKey();
}

// configure topology
void MeshNode::AddRoute(MeshRoute inRoute)
{
    mRoutes.push_back(inRoute);
}

bool MeshNode::HasNeighbor(HGID inNeighbor) const
{
    bool found = false;
    for (auto route = mRoutes.begin(); !found && route != mRoutes.end(); ++route)
    {
        found = route->back() == inNeighbor && route->size() == 1;
    }
    return found;
}

void MeshNode::AddNeighbor(HGID inNeighbor)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "AddNeighbor " << inNeighbor << endl;

    bool found = HasNeighbor(inNeighbor);

    if (found == false) {
        mRoutes.push_back(MeshRoute(inNeighbor, 1));

        // initiate a payment channel if one doesn't already exist with this neighbor
        ProposeChannel(inNeighbor);
    }
}

// initiate a payment channel if one doesn't already exist with this neighbor
void MeshNode::ProposeChannel(HGID inNeighbor)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "ProposeChannel to " << inNeighbor << endl;

    PeerChannel theChannel;
    theChannel.mFundingPeer = inNeighbor;
    theChannel.mProposingPeer = HGID();
    theChannel.mUnspentTokens = 100;
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

    UpdateIncentiveHeader(theMessage);

    SendTransmission(theMessage);
}

// get existing channel
PeerChannel& MeshNode::GetChannel(HGID inNeighbor)
{
    for (auto channel = mPeerChannels.begin(); channel != mPeerChannels.end(); ++channel) {
        if (channel->mFundingPeer == HGID() || channel->mProposingPeer == HGID()) {
            return *channel;
        }
    }
    throw std::invalid_argument("No channel exists for neighbor.");
}

// originate new message
bool MeshNode::OriginateMessage(const HGID inDestination, const std::string &inPayload)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "OriginateMessage to " << inDestination << ": " << inPayload << endl;
    
    MeshMessage theMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = GetNextHop(inDestination);
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inDestination;
    theMessage.mPayloadData = inPayload;
    UpdateIncentiveHeader(theMessage);

    SendTransmission(theMessage);
}

// relay a message
bool MeshNode::RelayMessage(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "RelayMessage: " << inMessage << endl;

    // receive payment
    {
        uint8_t commited_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
        PeerChannel &theChannel = GetChannel(inMessage.mSender);
        theChannel.mUnspentTokens -= commited_tokens;
        theChannel.mSpentTokens += commited_tokens;
        theChannel.mLastNonce += 1;
        theChannel.mState = inMessage.mIncentive.mType;
        theChannel.mConfirmed = false;
    }

    // new relay message
    MeshMessage outMessage = inMessage;
    outMessage.mSource = GetHGID();
    outMessage.mReceiver = GetNextHop(outMessage.mDestination);
    UpdateIncentiveHeader(outMessage);

    // send payment
    {
        uint8_t commited_tokens = (outMessage.mIncentive.mPrepaidTokens - (outMessage.mIncentive.mRelayPath.size() + 1));
        PeerChannel &theChannel = GetChannel(outMessage.mReceiver);
        theChannel.mUnspentTokens += commited_tokens;
        theChannel.mSpentTokens -= commited_tokens;
        theChannel.mLastNonce += 1;
        theChannel.mState = inMessage.mIncentive.mType;
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
    
    // create channel entry
    // TODO: check that funds exist, etc.
    PeerChannel theChannel;
    theChannel.mFundingPeer = HGID();
    theChannel.mProposingPeer = inMessage.mReceiver;
    theChannel.mUnspentTokens = 100;
    theChannel.mSpentTokens = 0;
    theChannel.mLastNonce = 0;
    theChannel.mState = eSetup2;
    theChannel.mConfirmed = false;
    return true;
}

// receive message
bool MeshNode::ReceiveMessage(const MeshMessage& inMessage)
{    
    cout << "Node " << GetHGID() << ", ";
    cout << "ReceiveMessage: " << inMessage << endl;

    // destination node confirms message hash matches
    if (!VerifyMessage(inMessage)) {
        return false;
    }

    // receive payment
    uint8_t remaining_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
    theChannel.mUnspentTokens -= remaining_tokens;
    theChannel.mSpentTokens += remaining_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = eReceipt1;
    theChannel.mConfirmed = false;

    // send return receipt
    MeshMessage theMessage = inMessage;
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inMessage.mSender;
    theMessage.mSource = GetHGID();
    theMessage.mDestination = inMessage.mSource;
    UpdateIncentiveHeader(theMessage);

    // send proof of receipt to previous hop
    SendTransmission(theMessage);

    return true;
}

bls::Signature MeshNode::GetAggregateSignature(const MeshMessage& inMessage)
{
    cout << "GetAggregateSignature, " << endl;
    // calculate aggregation info from implied transaction hashes and signing public keys    
    std::vector<ImpliedTransaction> theTransactions = GetTransactions(inMessage);
    std::vector<bls::AggregationInfo> aggregation_info;
    for (const ImpliedTransaction &tx : theTransactions) {
        bls::PublicKey pk = bls::PublicKey::FromBytes(tx.mTransactionSigner.data());
        aggregation_info.push_back( bls::AggregationInfo::FromMsgHash(pk, tx.GetTransactionHash().data()) );
        cout << "\t tx: [";
        for (int v: tx.GetTransactionHash()) { std::cout << std::hex << v; }
        cout << "]" << endl;
        cout << "\tSigner PK: ";
        for (int v : tx.mTransactionSigner) { cout << std::hex << v; }
        cout << endl;
    }
    bls::AggregationInfo merged_aggregation_info = bls::AggregationInfo::MergeInfos(aggregation_info);
    bls::Signature agg_sig = bls::Signature::FromBytes(inMessage.mIncentive.mSignature.data());
    agg_sig.SetAggregationInfo(merged_aggregation_info);
    return agg_sig;
}

// compute implied transaction from message and peer channel
std::vector<ImpliedTransaction> MeshNode::GetTransactions(const MeshMessage& inMessage)
{
    std::vector<ImpliedTransaction> theTransactions;
    const L49Header& incentive = inMessage.mIncentive;
    uint8_t tokens = incentive.mPrepaidTokens;
    const MeshNode& destination = MeshNode::FromHGID(inMessage.mDestination);
    vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN);
    const MeshNode& source = MeshNode::FromHGID(inMessage.mSource);
    const MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);
    const MeshNode& first_relay = MeshNode::FromHGID(incentive.mRelayPath.empty() ? inMessage.mSender : incentive.mRelayPath.front());

    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

    ImpliedTransaction issued_value_tx = ImpliedTransaction::Issue(source.GetPublicKey(), 255);
    ImpliedTransaction setup_tx = ImpliedTransaction::Setup(issued_value_tx, source.GetPublicKey(), first_relay.GetPublicKey(), 10);
    ImpliedTransaction refund_tx = ImpliedTransaction::Refund(setup_tx, source.GetPublicKey(), first_relay.GetPublicKey(), 10);

    cout << "GetTransactions, Type: " << incentive.mType << endl;

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
        path.push_back(GetHGID());

        ImpliedTransaction last_update_tx = setup_tx;
        HGID hgid1 = inMessage.mSource;
        for (auto hgid2 = path.begin(); hgid2 != path.end(); ++hgid2) {
            const MeshNode& sender = MeshNode::FromHGID(hgid1);
            const MeshNode& receiver = MeshNode::FromHGID(*hgid2);
            // create update and settle tx
            theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(last_update_tx, sender.GetPublicKey(), receiver.GetPublicKey(), tokens, tokens-1, destination.GetPublicKey(), message_hash));
            hgid1 = *hgid2;
            tokens--;
            last_update_tx = theTransactions.back();
        }
    }
    if (incentive.mType >= eNegotiate2 ) {
        const MeshNode& sender = MeshNode::FromHGID(GetHGID());
        const MeshNode& receiver = MeshNode::FromHGID(inMessage.mDestination);
        // create update and settle tx for delivery to destination node
        theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(theTransactions.back(), sender.GetPublicKey(), receiver.GetPublicKey(), tokens, tokens-1, destination.GetPublicKey(), message_hash));
    }
    return theTransactions;
}

void MeshNode::UpdateIncentiveHeader(MeshMessage& ioMessage)
{
    const PeerChannel& theChannel = GetChannel(ioMessage.mReceiver);
    L49Header& incentive = ioMessage.mIncentive;
    incentive.mType = theChannel.mState;
    incentive.mPrepaidTokens = theChannel.mSpentTokens + theChannel.mUnspentTokens;
    if ((theChannel.mState == eNegotiate1 || theChannel.mState == eNegotiate2) && ioMessage.mSender != ioMessage.mSource) {
        incentive.mRelayPath.push_back(ioMessage.mSender);
    } 
    else if ((theChannel.mState == eReceipt1 || theChannel.mState == eReceipt2) && ioMessage.mSender != ioMessage.mDestination) {
        incentive.mRelayPath.push_back(ioMessage.mSender);
    }

    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(ioMessage);

    if (incentive.mType == eSetup1) {
        // initialize incentive signature by signing refund tx
        bls::Signature first_sig = SignTransaction(theImpliedTransactions.back());
        incentive.mSignature.resize(bls::Signature::SIGNATURE_SIZE);
        first_sig.Serialize(&incentive.mSignature[0]);
    }
    else {
        vector<bls::Signature> sigs;
        // push current signature contained in the message
        sigs.push_back( GetAggregateSignature(ioMessage));

        // sign setup/funding tx
        if (incentive.mType == eNegotiate1 && ioMessage.mSender == ioMessage.mSource) {
            assert(theImpliedTransactions.size() == 3);
            assert(theImpliedTransactions[1].GetType() == eSetup);
            sigs.push_back( SignTransaction(theImpliedTransactions[1]));
        }

        if (incentive.mType == eReceipt1) {
            assert(ioMessage.mSender == ioMessage.mDestination);
            sigs.push_back( SignMessage(ioMessage.mPayloadData));
        }

        // add signature for latest tx
        sigs.push_back( SignTransaction(theImpliedTransactions.back()));

        // update aggregate signature
        try {
            bls::Signature agg_sig = bls::Signature::AggregateSigs(sigs);
            agg_sig.Serialize(&incentive.mSignature[0]);
        }
        catch (std::string& e) {
            cout << e << endl;
        }
    }
}

//
bls::Signature MeshNode::SignTransaction(const ImpliedTransaction& inTransaction) const
{
    cout << "Node " << GetHGID() << ", ";
    cout << "SignTransaction, Tx Type: " << inTransaction.GetType() << " tx: [";
    for (int v: inTransaction.GetTransactionHash()) { std::cout << std::hex << v; } 
    cout << "]" << endl;
    cout << "\tTx Signer PK: ";
    for (int v : inTransaction.mTransactionSigner) { cout << std::hex << v; }
    cout << endl;
 
    std::vector<uint8_t> msg = inTransaction.Serialize();
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::PublicKey pk = sk.GetPublicKey();
    bls::Signature sig = sk.Sign(msg.data(), msg.size());
    return sig;
}

bls::Signature MeshNode::SignMessage(const std::string& inPayload) const
{
    cout << "Node " << GetHGID() << ", ";
    cout << "SignMessage: " << inPayload << endl;
 
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::Signature sig = sk.Sign(reinterpret_cast<const uint8_t*>(inPayload.data()), inPayload.size());
    return sig;
}

// transmit message
void MeshNode::SendTransmission(const MeshMessage& inMessage)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "SendTransmission: " << inMessage << endl;
    
    if (!VerifyMessage(inMessage)) {
        return;
    }
    MeshNode& receiver = MeshNode::FromHGID(inMessage.mReceiver);
    receiver.ReceiveTransmission(inMessage);
}

// receive message
void MeshNode::ReceiveTransmission(const MeshMessage &inMessage)
{
    cout << "Node " << GetHGID() << ", ";
    cout << "ReceiveTransmission" << inMessage << endl;
    MeshMessage theMessage = inMessage;

    // check that aggregate signature is valid
    MeshNode& sender = MeshNode::FromHGID(inMessage.mSender);
    if (!sender.VerifyMessage(inMessage)) {
        return;
    }

    // handle setup1 (fund channel)
    if (theMessage.mIncentive.mType == eSetup1) {
        FundChannel(theMessage);
        return;
    }

    // handle negotiate 1 (relay message)
    if (theMessage.mDestination != GetHGID()) {
        RelayMessage(theMessage);
        return;
    }

    // handle negotiate2 (recieve message)
    if (theMessage.mIncentive.mType == eNegotiate2) {
        ReceiveMessage(theMessage);
        return;
    }
}

// check that aggregate signature is valid
bool MeshNode::VerifyMessage(const MeshMessage& inMessage) 
{
    cout << "Node " << GetHGID() << ", ";
    cout << "VerifyMessage: " << inMessage << endl;
    cout << "\t";

    bls::Signature agg_sig = GetAggregateSignature(inMessage);
    if (!agg_sig.Verify()) {
        cout << "\tfailed!" << endl;
        return false;
    }
    cout << "\tsuccess!" << endl;
    return true;
}

}; // namespace lot49