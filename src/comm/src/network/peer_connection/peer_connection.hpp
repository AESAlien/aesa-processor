#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/session/session.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace comm
{

// A configured peer persists across TCP reconnections. Session snapshots keep
// an old connection alive while a sender or receiver is still using it.
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

    // Does nothing while disconnected. Throws SendError on send failure.
    // A peer has one sending thread, as required by Session.
    void send(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

    bool isConnected() const;

    // Returns the current session, or nullptr while disconnected. The copy
    // keeps the session alive even if it is replaced meanwhile.
    std::shared_ptr<Session> snapshotSession() const;

    // Replaces the active session when the configured peer reconnects.
    void replaceSession(std::shared_ptr<Session> session);

    // Receives only if this is still the active session; nullopt means stale.
    std::optional<std::vector<MessageFrame>> receiveIfCurrent(
        const std::shared_ptr<Session>& session
    );

private:
    void detachIfCurrent(const std::shared_ptr<Session>& session);

    std::string _name;
    CommEndpoint _endpoint;
    mutable std::mutex _mutex;
    std::shared_ptr<Session> _session;
};

} // namespace comm
