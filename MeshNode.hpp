using namespace std;

#include "bls.hpp"
#include "ImpliedTransaction.hpp"

#include <list>
#include <memory>

#pragma once

namespace lot49
{

// Hashed GID
typedef uint16_t HGID;

// Mesh route
typedef std::vector<HGID> MeshRoute;

//
// incentive headers
//

enum EChannelState 
{
    eSetup1,
    eSetup2,
    eNegotiate1,
    eNegotiate2,
    eNegotiate3,
    eReceipt1,
    eReceipt2,
    eClose1,
    eClose2
};

struct PeerChannel
{
    HGID mFundingPeer;
    HGID mProposingPeer;
    uint8_t mUnspentTokens;
    uint8_t mSpentTokens;
    uint16_t mLastNonce; // used to create unique shared 2-of-2 address
    EChannelState mState; // Setup1 -> Setup2 -> Negotiate1 -> Negotiate2 -> Receipt1 -> Receipt2 -> (Close1 -> Close2) or (back to Negotiate1)
    bool mConfirmed; // setup tx confirmed
};

static const uint8_t MAXRELAYS = 8;

struct L49Header
{
    EChannelState mType;
    uint8_t mPrepaidTokens;
    std::vector<HGID> mRelayPath; // lot49::MAXRELAYS
    std::vector<uint8_t> mSignature; // bls::Signature::SIGNATURE_SIZE
};

//
// Mesh Message
//

struct MeshMessage
{
    HGID mSender;
    HGID mReceiver;
    HGID mSource;
    HGID mDestination;
    L49Header mIncentive;

    // payload, max 236 bytes
    std::string mPayloadData;
};

enum EEventType
{
    eOriginate,
    eTransmit,
    eReceive
};

struct MessageEvent
{
    EEventType mEvent;
    MeshMessage mMessage;
    uint32_t mTimestamp;
    uint16_t mNonce; // recreate payment address using mMessage.mSenderHGID + mMessage.mReceiverHGID + mNonce
};

class MeshNode
{
  public:

    // create mesh nodes
    static void CreateNodes(const int inCount);

    static MeshNode &FromIndex(const int inIndex);

    // Lookup or construct a node from a Hashed GID
    static MeshNode &FromHGID(const HGID &inHGID);

    HGID GetHGID() const;
    HGID GetNextHop(HGID inDestination) const;

    // access public key
    const bls::PublicKey GetPublicKey() const;

    // configure topology
    void AddRoute(MeshRoute inRoute);
    
    bool HasNeighbor(HGID inNeighbor) const;

    // propose channel to new neighbor
    void AddNeighbor(HGID inNeighbor);

    // originate new message
    bool OriginateMessage(const HGID inDestination, const std::string &inPayload);

  private:
    MeshNode();

    // get existing peer channel open with neighbor 
    PeerChannel& GetChannel(HGID inNeighbor);

    // open a channel with neighbor node
    void ProposeChannel(HGID inNeighbor);

    //
    bls::Signature GetAggregateSignature(const MeshMessage& inMessage);

    // 
    std::vector<ImpliedTransaction> GetTransactions(const MeshMessage& inMessage);

    // 
    void UpdateIncentiveHeader(MeshMessage& ioMessage);

    // 
    bls::Signature SignTransaction(const ImpliedTransaction& inTransaction) const;

    // destination node signs payload 
    bls::Signature SignMessage(const std::string& inPayload) const;

    // transmit message
    void SendTransmission(const MeshMessage& inMessage);

    // receive message
    void ReceiveTransmission(const MeshMessage& inMessage);

    // check that aggregate signature is valid
    bool VerifyMessage(const MeshMessage &inMessage);

    // relay a message
    bool RelayMessage(const MeshMessage& inMessage);

    // fund a channel
    bool FundChannel(const MeshMessage& inMessage);

    // receive message
    bool ReceiveMessage(const MeshMessage& inMessage);

    static vector<MeshNode> sNodes;

    std::list<MessageEvent> mMessageEvents;
    std::list<PeerChannel> mPeerChannels;

    // routes computed by routing protocol
    std::list<MeshRoute> mRoutes;

    std::vector<uint8_t> mSeed;
};

}; // namespace lot49