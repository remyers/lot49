#include "bls.hpp"
#include "MeshNode.hpp"
#include <algorithm>

using namespace std;
using namespace lot49;

// TODO: use  32 byte signatures, not 96 byte: https://twitter.com/sorgente711/status/1025451092610433024

bool testBLS();

void TestRoute(HGID inSender, HGID inReceiver, std::string& inMessage)
{
    // send a message and receive delivery receipt
    cout << std::hex << endl << "Node " << std::setw(4) << std::setfill('0') << inSender << ": send a message to Node " << std::setw(4) << inReceiver << endl; 
    MeshNode::FromHGID(inSender).OriginateMessage(inReceiver, std::vector<uint8_t>(inMessage.begin(), inMessage.end()));
}

int main(int argc, char* argv[]) 
{
    cout << "testBLS():  " << (testBLS() ? "success!" : "failed!") << endl;
  
    const size_t MAX_NODES = 5; 
    MeshNode::CreateNodes(MAX_NODES);
 
    // create linear route: A <-> B <-> C .. etc
    MeshRoute route1;
    for (int i = 0; i < MAX_NODES-1; i++) {
        HGID hgid = MeshNode::FromIndex(i).GetHGID();
        route1.push_back(hgid);
    }
    MeshRoute route2 = route1;
    route2.pop_back();
    HGID hgid = MeshNode::FromIndex(MAX_NODES-1).GetHGID();
    route2.push_back(hgid);

    // add route from first to D1 node A->B->C->D1
    MeshNode::AddRoute(route1);

    // add route from first to D2 node A->B-C->D2
    MeshNode::AddRoute(route2);

    // set the last node as a gateway for verifying setup transactions
    MeshNode::AddGateway(route2.back());

    cout << "----------------------------------------------" << endl << endl;

    // send a message from route 0 [A to C], and receive delivery receipt
    std::string payload = "TEST TEST TEST";
    TestRoute(route1[0], route1[2], payload);

    cout << "----------------------------------------------" << endl << endl;

    // send a message from route 1 [A to D1], and receive delivery receipt
    payload = "Mr. Watson - come here - I want to see you.";
    TestRoute(route1.front(), route1.back(), payload);

    cout << "----------------------------------------------" << endl << endl;

    // send a message from route 2 [A to D2], and receive delivery receipt
    payload = "The Times 03/Jan/2009 Chancellor on brink of second bailout for banks.";
    TestRoute(route2.front(), route2.back(), payload);

    cout << "----------------------------------------------" << endl << endl;

    // send a message from reverse of route 1 [D2 to A], and receive delivery receipt
    payload = "The computer can be used as a tool to liberate and protect people, rather than to control them.";
    TestRoute(route2.back(), route2.front(), payload);
};

bool testBLS()
{
    //
    // Creating keys and signatures
    //

    // Example seed, used to generate private key. Always use
    // a secure RNG with sufficient entropy to generate a seed.
    uint8_t seed[] = {0, 50, 6, 244, 24, 199, 1, 25, 52, 88, 192,
                      19, 18, 12, 89, 6, 220, 18, 102, 58, 209,
                      82, 12, 62, 89, 110, 182, 9, 44, 20, 254, 22};

    bls::PrivateKey sk = bls::PrivateKey::FromSeed(seed, sizeof(seed));
    bls::PublicKey pk = sk.GetPublicKey();

    //
    // Serializing keys and signatures to bytes
    //

    uint8_t skBytes[bls::PrivateKey::PRIVATE_KEY_SIZE]; // 32 byte array
    uint8_t pkBytes[bls::PublicKey::PUBLIC_KEY_SIZE];   // 96 byte array
    uint8_t sigBytes[bls::Signature::SIGNATURE_SIZE];   // 48 byte array

    pk.Serialize(pkBytes); // 96 bytes

    // Takes array of 96 bytes
    pk = bls::PublicKey::FromBytes(pkBytes);

    sk.Serialize(skBytes); // 32 bytes

    uint8_t msg[] = {100, 2, 254, 88, 90, 45, 23};
    bls::Signature sig = sk.Sign(msg, sizeof(msg));

    sig.Serialize(sigBytes); // 48 bytes

    //
    // Loading keys and signatures from bytes
    //

    // Takes array of 32 bytes
    sk = bls::PrivateKey::FromBytes(skBytes);

    // Takes array of 96 bytes
    pk = bls::PublicKey::FromBytes(pkBytes);

    // Takes array of 48 bytes
    sig = bls::Signature::FromBytes(sigBytes);

    //
    // Verifying signatures
    //

    // Add information required for verification, to sig object
    sig.SetAggregationInfo(bls::AggregationInfo::FromMsg(pk, msg, sizeof(msg)));

    bool ok = sig.Verify();

    //
    // Aggregate signatures for a single message
    //

    {
        // Generate some more private keys
        seed[0] = 1;
        bls::PrivateKey sk1 = bls::PrivateKey::FromSeed(seed, sizeof(seed));
        seed[0] = 2;
        bls::PrivateKey sk2 = bls::PrivateKey::FromSeed(seed, sizeof(seed));

        // Generate first sig
        bls::PublicKey pk1 = sk1.GetPublicKey();
        bls::Signature sig1 = sk1.Sign(msg, sizeof(msg));

        // Generate second sig
        bls::PublicKey pk2 = sk2.GetPublicKey();
        bls::Signature sig2 = sk2.Sign(msg, sizeof(msg));

        // Aggregate signatures together
        vector<bls::Signature> sigs = {sig1, sig2};
        bls::Signature aggSig = bls::Signature::AggregateSigs(sigs);

        // For same message, public keys can be aggregated into one.
        // The signature can be verified the same as a single signature,
        // using this public key.
        vector<bls::PublicKey> pubKeys = {pk1, pk2};

        bls::PublicKey aggPubKey = bls::PublicKey::Aggregate(pubKeys);
    }

    //
    // Aggregate signatures for different messages
    //

    // Generate one more key and message
    seed[0] = 1;
    bls::PrivateKey sk1 = bls::PrivateKey::FromSeed(seed, sizeof(seed));
    bls::PublicKey pk1 = sk1.GetPublicKey();

    seed[0] = 2;
    bls::PrivateKey sk2 = bls::PrivateKey::FromSeed(seed, sizeof(seed));
    bls::PublicKey pk2 = sk2.GetPublicKey();

    seed[0] = 3;
    bls::PrivateKey sk3 = bls::PrivateKey::FromSeed(seed, sizeof(seed));
    bls::PublicKey pk3 = sk3.GetPublicKey();
    uint8_t msg2[] = {100, 2, 254, 88, 90, 45, 23};

    // Generate the signatures, assuming we have 3 private keys
    bls::Signature sig1 = sk1.Sign(msg, sizeof(msg));
    bls::Signature sig2 = sk2.Sign(msg, sizeof(msg));
    bls::Signature sig3 = sk3.Sign(msg2, sizeof(msg2));

    // They can be noninteractively combined by anyone
    // Aggregation below can also be done by the verifier, to
    // make batch verification more efficient
    vector<bls::Signature> sigsL = {sig1, sig2};
    bls::Signature aggSigL = bls::Signature::AggregateSigs(sigsL);

    // Arbitrary trees of aggregates
    vector<bls::Signature> sigsFinal = {aggSigL, sig3};
    bls::Signature aggSigFinal = bls::Signature::AggregateSigs(sigsFinal);

    // Serialize the final signature
    aggSigFinal.Serialize(sigBytes);

    //
    // Verify aggregate signature for different messages
    //

    // Deserialize aggregate signature
    aggSigFinal = bls::Signature::FromBytes(sigBytes);

    // Create aggregation information (or deserialize it)
    bls::AggregationInfo a1 = bls::AggregationInfo::FromMsg(pk1, msg, sizeof(msg));
    bls::AggregationInfo a2 = bls::AggregationInfo::FromMsg(pk2, msg, sizeof(msg));
    bls::AggregationInfo a3 = bls::AggregationInfo::FromMsg(pk3, msg2, sizeof(msg2));
    vector<bls::AggregationInfo> infos = {a1, a2};
    bls::AggregationInfo a1a2 = bls::AggregationInfo::MergeInfos(infos);
    vector<bls::AggregationInfo> infos2 = {a1a2, a3};
    bls::AggregationInfo aFinal = bls::AggregationInfo::MergeInfos(infos2);

    // Verify final signature using the aggregation info
    aggSigFinal.SetAggregationInfo(aFinal);
    ok = aggSigFinal.Verify();

    // If you previously verified a signature, you can also divide
    // the aggregate signature by the signature you already verified.
    ok = aggSigL.Verify();
    vector<bls::Signature> cache = {aggSigL};
    aggSigFinal = aggSigFinal.DivideBy(cache);

    // Final verification is now more efficient
    ok = aggSigFinal.Verify();

    return ok;
}