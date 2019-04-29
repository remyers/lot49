# Transmission Overhead

In this appendix we analyze the transmission overhead of three smart contract systems for paying incentives to mobile mesh nodes for successful message delivery. This analysis focuses on the communication overhead to update channel states and close channels. We assume the channel setup overhead is roughly similar for all three. We also assume the same simplifying optimizations have been made to all three systems to minimize communication overhead. For each system we analyze the total communication overhead needed to update the payment channels of an example three hop message delivery. We also examine the amount of data that must be communicated to the internet to close all three channels.

The three smart contract systems are based on the following protocols:

* Poon-Dryja state revocation (with ECDSA signatures)
* eltoo state revocation (with Schnorr multisignatures)
* eltoo state revocation (with BLS non-interactive signature aggregation)

The Poon-Dryja state revocation protocol requires multiple communication phases for both the sender and receiver of a payment to exchange commitments to the new state and revoke the previous state. The eltoo state revocation protocol uses transaction replacement and only requires the payment sender to commit to a new channel state. In the eltoo/Schnorr variant all nodes involved in the message delivery sign a single transaction that updates the channels along the route. In the eltoo/BLS variant each node signs a transaction to update their channel with the next node in the route.

## Simplifying Assumptions:

We assume all three systems use the following simplifications and optimizations to reduce the bandwidth required to update their channel state.
 
* Each node has a short 2-byte node ID.
* The extended public key for each node ID is pre-shared.
* New public keys are deterministically derived from the extended public key for each transaction.
* A route of relays nodes from the sender to a destination node are computed by the sender and communicated in clear text (no onion routing).
* Transactions, including script details and defaults, can be inferred from a 1 byte type ID.
* The destination node signs a hash of the message to confirm receipt and unlock channel updates between nodes.

## Lightning Protocol

For this analysis, the following data structures are used to compute the overhead of simplified versions of the protocol messages described in “[Bolt #2](https://github.com/lightningnetwork/lightning-rfc/blob/master/02-peer-protocol.md) : Peer Protocol For Channel Management.”

### update_add_htlc:
|       | LN Size (bytes) | Simplified Size (bytes) | Note |
| ----- | ----                  | ----              | ---- |
| type | 1 | 1 | 128 (update_add_htlc) |
| channel_id | 32 | - | inferred from destination node |
| id | 8 |  - | ID of the sender is included in the normal mesh routing header |
| amount_msat | 8 | 2 | smaller max payment range |
| payment_hash | 32 | - | inferred from message data |
| cltv_expiry | 4 | - | use standard value |
| onion_routing_packet| 1366 | 12 | up to 6 clear text hops, one 2 byte node ID per hop |


### commitment_signed:
|       | LN Size (bytes) | Simplified Size (bytes) | Note |
| ----- | ----                  | ----              | ---- |
| type | 1 | 1 132 (commitment_signed) |
| channel_id | 32 | - | inferred from destination node |
| signature | 64 | 64 | sign remote commitment (?) | 
| num_htlcs | 2 | - | assume single htlc signature |
| htlc_signature | num_htlcs*64 | 64 | commit to previous payments (?) |

### revoke_and_ack:
|       | LN Size (bytes) | Simplified Size (bytes) | Note |
| ----- | ----                  | ----              | ---- |
| type | 1 | 1 | 132 (commitment_signed) | 
| channel_id | 32 | - | inferred from destination node |
| per_commitment_secret | 32 | 32 | revoke last commitment |
| next_per_commitment_point | 33 | 33 | new commitment point |

### closing_signed:
|       | LN Size (bytes) | Simplified Size (bytes) | Note |
| ----- | ----                  | ----              | ---- |
| type | 1 | 1 | 39 (closing_signed) |
| channel_id | 32 | - | inferred from destination node |
| fee_satoshis | 8 | 2 | smaller max amount |
| signature | 64 | 64 | |

Both the sender and receiver must commit to a new channel state and revoke the previous channel state, so to update three channels requires transmitting each of these messages six times. 

Unfortunately a Schnorr multisignature can not be used to reduce the overall amount of data transmitted between nodes. Signatures are part of both the commitment_signed and closing_signed messages. However, because nodes update their channel state by transmitting only their own signature, no savings is seen if the initial setup transaction uses a multisignature. Both parties signatures are only combined when a channel is closed cooperatively by signing a partially signed closing transaction or uncooperatively by signing an unrevoked HTLC transaction. 

| Message Type | Simplified Size (bytes) | Total Update* (bytes) ECDSA or Schnorr | Total Close** (bytes) ECDSA |
| ------------ | --------------- | -------------------------------- | ------------------------------- | 
| update_add_htlc | 15 | 90 | - |
| commitment_signed | 129 | 774 | - |
| revoke_and_ack | 66 | 396 | -
| closing_signed | 67 | - | 201 |
| Total: | | 1260 | 201 |

\* Total data transmitted by both sender and receiver to Update three channels

\*\* Total data transmitted by both sender and receiver to Close three channels

## Lot49 Protocol

The Lot49 proposal uses the eltoo scheme to update channel states. There is no revocation phase in this update protocol which reduces the amount of data transmitted. It also means that nodes that are part of the same transaction chain can cooperatively sign a single combined transaction to update their respective channel states. In our example the transaction chain includes the transactions that update the three channel states between the message sender and destination node. If an out-of-date update is committed by one of the nodes, it will not result in a penalty and can be replaced by any of the nodes by committing a more recent update within the timelock period. 

### update (update_add_htlc & commitment_signed):
| | Size Schnorr (bytes) | Size BLS (bytes) | Note |
| ------- | -------- | ------- | ------- |
| type | 1 | 1 | 2 (eNegotiate1) |
| channel_id | - | - | inferred from destination node |
| id | - | - | ID of the sender is included in the normal mesh routing header |
| prepaid_tokens (amount_msat) | 1 | 1 | sat amount committed by message sender |
| payment_hash | - | - | inferred from message data | 
| cltv_expiry | - | - | use standard value
| relay_path (onion_routing_packet) | 12 | 12 | up to 6 clear text hops, one 2 byte node ID per hop; also used to reconstruct the complete transaction chain |
| agg_signature | 64 | 48 | commit to new transaction chain

### close (closing_signed):
| | Size Schnorr (bytes) | Size BLS (bytes) | Note |
| ------- | -------- | ------- | ------- |
| type | 1 | 1 | 7 (eClose1) |
| channel_id | - | - | inferred from destination node |
| id | - | - | ID of the sender is included in the normal mesh routing header |
| prepaid_tokens (amount_msat) | 1 | 1 | msat committed by message sender |
| payment_hash | - | - | inferred from message data |
| cltv_expiry | - | - | use standard value |
| relay_path (onion_routing_packet) | 12 | 12  |up to 6 clear text hops, one 2 byte node ID per hop; also used to reconstruct the complete transaction chain |
| agg_signature | 64 | 48 | commit to new transaction chain |

All of the nodes along the relay path collaboratively sign a single transaction (with Schnorr) or set of transactions (BLS). This involves each node modifying the update message to aggregate their signature with the agg_signature field data they received before retransmitting it to the next relay node. To update all three channels requires transmitting the update message three times. 

A Schnorr multisignature or BLS aggregate signature is used to reduce the total amount of data transmitted between nodes. If using Schnorr signatures, each node aggregates their signature to commit to spend from a single setup transaction that must be signed by all of the involved relay nodes. If using BLS signatures, each node aggregates their signature to commit to spend from a setup transaction established with the next relay node along the route.

For each update, nodes do not need to revoke the previous channel state. This reduces the amount of signature information that must be transmitted. If at any point a relay becomes unresponsive, a single transaction signed with the last complete multisignature (Schnorr) or aggregate signature (BLS) can be committed to settle all of the channels involved.

| Message Type | Size Schnorr (bytes) | Size BLS (bytes) | Total Update* (bytes) Schnorr | Total Update* (bytes) BLS | Total Close** (bytes) Schnorr | Total Close** (bytes) BLS |
| -------------| -------------------- | ---------------| ---------- | ---------- | ---------- | ---------- | 
| update | 78 | 62 | 234 | 189 | - | - |
| close | 78 | 62 | - | - | 78 | 62 |
| | | Total: | 234 | 189 | 78 | 62 |

\* Total data transmitted by both sender and receiver to Update three channels

\*\* Total data transmitted by both sender and receiver to Close three channels

## Conclusion

For an example three hop message delivery, the eltoo revocation system would require transmitting 1/5 as much data overall to update channels (234 vs 1260 bytes) compared to the Poon-Dryja update scheme used by the Lightning Network. Signature aggregation could reduce the amount of data that must be transmitted to an internet gateway to close channels by almost a third (78 vs 201 bytes). The reduction in transmission overhead from using eltoo and signature aggregation increases proportionally for longer relay paths.
