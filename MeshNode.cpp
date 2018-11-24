
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

// create mesh nodes
void MeshNode::CreateNodes(const size_t inCount)
{
    sNodes.resize(inCount);
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

MeshNode &MeshNode::FromIndex(const size_t inIndex)
{
    if (inIndex > sNodes.size())
    {
        CreateNodes(inIndex + 1);
    }
    return sNodes[inIndex];
}

// ctor
MeshNode::MeshNode()
{
    std::generate_n(mSeed, 32, [&] { return dist(rng); });
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
    theMessage.mSender = GetHGID();
    theMessage.mReceiver = inNeighbor;
    theMessage.mDestination = inNeighbor;
    UpdateIncentiveHeader(theMessage);
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
    // receive payment
    uint8_t remaining_tokens = (inMessage.mIncentive.mPrepaidTokens - inMessage.mIncentive.mRelayPath.size());
    PeerChannel& theChannel = GetChannel(inMessage.mSender);
    theChannel.mUnspentTokens -= remaining_tokens;
    theChannel.mSpentTokens += remaining_tokens;
    theChannel.mLastNonce += 1;
    theChannel.mState = inMessage.mIncentive.mType;
    theChannel.mConfirmed = false;
 
    // new relay message
    MeshMessage theMessage = inMessage;
    theMessage.mSource = GetHGID();
    theMessage.mReceiver = GetNextHop(theMessage.mDestination);
    UpdateIncentiveHeader(theMessage);

    // send message to next hop
    SendTransmission(theMessage);

    return true;
}

// fund a channel
bool MeshNode::FundChannel(const MeshMessage& inMessage)
{    
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

// compute implied transaction from message and peer channel
std::vector<ImpliedTransaction>& MeshNode::GetTransactions(const MeshMessage& inMessage, const PeerChannel& inChannel)
{
    std::vector<ImpliedTransaction> theTransactions;
    const L49Header& incentive = inMessage.mIncentive;
    uint8_t tokens = incentive.mPrepaidTokens;
    const MeshNode& destination = MeshNode::FromHGID(inMessage.mDestination);
    vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN);
    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(inMessage.mPayloadData.data()), inMessage.mPayloadData.size());

    if (incentive.mType >= eSetup1) {
        const MeshNode& source = MeshNode::FromHGID(inMessage.mSource);
        // create refund tx
        theTransactions.push_back( ImpliedTransaction::Refund(GetPublicKey(), source.GetPublicKey(), incentive.mPrepaidTokens));
    }
    if (incentive.mType >= eSetup2) {
        const MeshNode& source = MeshNode::FromHGID(inMessage.mSource);
        // create funding tx
        theTransactions.push_back( ImpliedTransaction::Setup(GetPublicKey(), source.GetPublicKey(), incentive.mPrepaidTokens));
    }
    if (incentive.mType >= eNegotiate1) {       
        // append current node to path
        vector<HGID> path = incentive.mRelayPath;
        path.push_back(GetHGID());

        HGID hgid1 = inMessage.mSource;
        for (auto hgid2 = path.rbegin(); hgid2 != path.rend(); ++hgid2) {
            const MeshNode& sender = MeshNode::FromHGID(hgid1);
            const MeshNode& receiver = MeshNode::FromHGID(*hgid2);
            // create update and settle tx
            theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(sender.GetPublicKey(), receiver.GetPublicKey(), inChannel.mLastNonce, tokens, tokens-1, destination.GetPublicKey(), message_hash));
            hgid1 = *hgid2;
            tokens--;
        }
    }
    if (incentive.mType >= eNegotiate2 ) {
        const MeshNode& sender = MeshNode::FromHGID(GetHGID());
        const MeshNode& receiver = MeshNode::FromHGID(inMessage.mDestination);
        // create update and settle tx for delivery to destination node
        theTransactions.push_back( ImpliedTransaction::UpdateAndSettle(sender.GetPublicKey(), receiver.GetPublicKey(), inChannel.mLastNonce, tokens, tokens-1, destination.GetPublicKey(), message_hash));
    }
}

void MeshNode::UpdateIncentiveHeader(MeshMessage& ioMessage)
{
    const PeerChannel& theChannel = GetChannel(ioMessage.mReceiver);
    std::vector<ImpliedTransaction> theImpliedTransactions = GetTransactions(ioMessage, theChannel);
    L49Header& incentive = ioMessage.mIncentive;

    incentive.mType = theChannel.mState;
    incentive.mPrepaidTokens = theChannel.mSpentTokens + theChannel.mUnspentTokens;
    if ((theChannel.mState == eNegotiate1 || theChannel.mState == eNegotiate2) && ioMessage.mSender != ioMessage.mSource) {
        incentive.mRelayPath.push_back(ioMessage.mSender);
    } 
    else if ((theChannel.mState == eReceipt1 || theChannel.mState == eReceipt2) && ioMessage.mSender != ioMessage.mDestination) {
        incentive.mRelayPath.push_back(ioMessage.mSender);
    }

    if (incentive.mType == eSetup1) {
        // initialize incentive signature by signing refund tx
        bls::Signature first_sig = SignTransaction(theImpliedTransactions.back());
        first_sig.Serialize(&incentive.mSignature[0]);
    }
    else {
        vector<bls::Signature> sigs;
        // push current signature
        sigs.push_back( bls::Signature::FromBytes(incentive.mSignature.data()));

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
        bls::Signature agg_sig = bls::Signature::AggregateSigs(sigs);
        agg_sig.Serialize(&incentive.mSignature[0]);
    }
}

//
bls::Signature MeshNode::SignTransaction(const ImpliedTransaction& inTransaction) const
{
    std::vector<uint8_t> msg = inTransaction.Serialize();
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::Signature sig = sk.Sign(msg.data(), msg.size());
    return sig;
}

bls::Signature MeshNode::SignMessage(const std::string& inPayload) const
{
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(mSeed.data(), mSeed.size());
    bls::Signature sig = sk.Sign(reinterpret_cast<const uint8_t*>(inPayload.data()), inPayload.size());
    return sig;
}

// transmit message
void MeshNode::SendTransmission(const MeshMessage& inMessage)
{
    MeshNode& receiver = MeshNode::FromHGID(inMessage.mReceiver);
    receiver.ReceiveMessage(inMessage);
}

// receive message
void MeshNode::ReceiveTransmission(const MeshMessage &inMessage)
{
    MeshMessage theMessage = inMessage;

    // check that aggregate signature is valid
    if (!VerifyMessage(inMessage)) {
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

}

}; // namespace lot49