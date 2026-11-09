#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/session/session.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace comm
{

// A configured peer persists across TCP reconnections and holds its current
// session. Sessions are replaced and cleared only on the receiving thread;
// the sending thread only reads them. A session copy keeps an old connection
// alive while a sender is still using it.
class PeerConnection
{
public:
    PeerConnection(std::string name, CommEndpoint endpoint);

    PeerConnection(const PeerConnection&) = delete;
    PeerConnection& operator=(const PeerConnection&) = delete;
    PeerConnection(PeerConnection&&) = delete;
    PeerConnection& operator=(PeerConnection&&) = delete;

    const std::string& getName() const noexcept;
    bool matches(const CommEndpoint& remoteEndpoint) const;
    // Compares the session's remote endpoint with this peer's endpoint.
    bool matches(const Session& session) const;

    // Does nothing while disconnected. Throws SendError on send failure.
    // A peer has one sending thread, as required by Session.
    void send(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

    // Receives once from the current session and returns the completed frames.
    // Clears the session when the connection has ended. Returns no frames while
    // disconnected. Throws ReceiveError. Called on the receiving thread only.
    std::vector<MessageFrame> receive();

    // Returns the current session, or nullptr while disconnected.
    std::shared_ptr<Session> getSession() const;

    // Replaces the current session; nullptr clears it. The previous session
    // is released outside the lock. Called on the receiving thread only.
    void setSession(std::shared_ptr<Session> session);

private:
    std::string _name;
    CommEndpoint _endpoint;
    mutable std::mutex _mutex;
    std::shared_ptr<Session> _session;
};

} // namespace comm
