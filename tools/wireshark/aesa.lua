-- Wire layout: src/comm/src/protocol/ST_*.hpp (packed, little-endian hosts).
-- block_size includes the 20-byte common header.
local aesa = Proto("aesa", "AESA Processor")
local HEADER_SIZE = 20
local MAXIMUM_FRAME_SIZE = 64 * 1024 -- MessageFramer default
local MAXIMUM_ITEM_COUNT = 100
local MESSAGE_NAMES = {
    [0xCA01] = "Operator control",
    [0xAC01] = "Track information",
    [0xAC02] = "Radar attitude to console",
    [0xAC03] = "Track deletion",
    [0xAB01] = "Beam transmit",
    [0xBA01] = "Plot information",
    [0xBA02] = "Radar attitude from simulator"
}
local BEAM_TYPES = {[1] = "Search", [2] = "Confirmation", [3] = "Tracking"}
local TRACK_TYPES = {[1] = "Init", [2] = "Tracking"}

local fields = {}
local function field(kind, name, label, ...)
    local value = ProtoField[kind]("aesa." .. name, label, ...)
    fields[#fields + 1] = value
    return value
end

local f = {
    messageId = field("uint16", "message_id", "Message ID", base.HEX, MESSAGE_NAMES),
    version = field("uint8", "version", "Version", base.DEC),
    blockSize = field("uint32", "block_size", "Block size (bytes, including header)", base.DEC),
    timeSec = field("uint32", "time_sec", "Time seconds", base.DEC),
    timeNsec = field("uint32", "time_nsec", "Time nanoseconds", base.DEC),
    sourceId = field("uint8", "source_id", "Source ID", base.DEC),
    destId = field("uint8", "dest_id", "Destination ID", base.DEC),
    padding = field("bytes", "padding", "Padding / reserved"),
    raw = field("bytes", "raw", "Uninterpreted data"),
    operation = field("uint8", "operation", "Operation", base.DEC, {[0] = "Off", [1] = "On"}),
    beamType = field("uint8", "beam.type", "Beam type", base.DEC, BEAM_TYPES),
    beamTimestamp = field("uint32", "beam.timestamp_ms", "Beam timestamp (ms)", base.DEC),
    beamId = field("uint32", "beam.id", "Beam ID", base.DEC),
    commandCount = field("uint32", "beam.command_count", "Command count", base.DEC),
    plotCount = field("uint16", "plot.count", "Plot count", base.DEC),
    plotId = field("uint32", "plot.id", "Plot ID", base.DEC),
    trackCount = field("uint16", "track.count", "Track count", base.DEC),
    trackId = field("uint16", "track.id", "Track ID", base.DEC),
    trackType = field("uint8", "track.type", "Track status", base.DEC, TRACK_TYPES),
    memoryFlag = field("uint8", "track.memory_flag", "Memory flag", base.DEC),
    trackTimestamp = field("uint32", "track.timestamp_ms", "Track timestamp (ms)", base.DEC),
    trackBeamId = field("uint32", "track.beam_id", "Track beam ID", base.DEC),
    trackCommandCount = field("uint32", "track.command_count", "Track command count", base.DEC),
    deletionCount = field("uint16", "deletion.count", "Deleted track count", base.DEC),
    deletedTrackId = field("uint16", "deletion.id", "Deleted track ID", base.DEC)
}

local function numericFields(kind, prefix, definitions)
    local result = {}
    for index, definition in ipairs(definitions) do
        result[index] = field(kind, prefix .. "." .. definition[1], definition[2])
    end
    return result
end

local beamFields = numericFields("float", "beam", {
    {"azimuth_deg", "Beam azimuth (deg)"}, {"elevation_deg", "Beam elevation (deg)"},
    {"azimuth_width_deg", "Azimuth width (deg)"}, {"elevation_width_deg", "Elevation width (deg)"}
})
local attitudeFields = numericFields("double", "attitude", {
    {"latitude_deg", "Radar latitude (deg)"}, {"longitude_deg", "Radar longitude (deg)"},
    {"altitude_km", "Radar altitude (km)"}, {"roll_deg", "Roll (deg)"},
    {"pitch_deg", "Pitch (deg)"}, {"yaw_deg", "Yaw (deg)"}
})
local plotFields = numericFields("float", "plot", {
    {"slant_range_km", "Slant range (km)"}, {"azimuth_deg", "Azimuth (deg)"},
    {"elevation_deg", "Elevation (deg)"}, {"doppler_mps", "Doppler velocity (m/s)"},
    {"power_db", "Power (dB)"}
})
local trackFields = numericFields("float", "track", {
    {"slant_range_km", "Slant range (km)"}, {"ground_range_km", "Ground range (km)"},
    {"azimuth_deg", "Azimuth (deg)"}, {"elevation_deg", "Elevation (deg)"},
    {"doppler_mps", "Doppler velocity (m/s)"}, {"latitude_deg", "Latitude (deg)"},
    {"longitude_deg", "Longitude (deg)"}, {"altitude_km", "Altitude (km)"},
    {"velocity_mps", "Velocity (m/s)"}, {"heading_deg", "Heading (deg)"},
    {"flight_path_angle_deg", "Flight path angle (deg)"}
})
aesa.fields = fields

local malformed = ProtoExpert.new(
    "aesa.malformed", "Invalid AESA message layout", expert.group.MALFORMED, expert.severity.ERROR
)
local unknownMessage = ProtoExpert.new(
    "aesa.unknown_message", "Unknown AESA message ID", expert.group.PROTOCOL, expert.severity.WARN
)
aesa.experts = {malformed, unknownMessage}
aesa.prefs.tcp_ports = Pref.range("TCP ports", "3200", "AESA server port(s)", 65535)

local function addNumericFields(tree, tvb, offset, values, size)
    for index, value in ipairs(values) do
        tree:add_le(value, tvb(offset + (index - 1) * size, size))
    end
end

local function requireSize(tree, tvb, size)
    if tvb:len() ~= size then
        tree:add_proto_expert_info(malformed,
            string.format("Expected %d bytes, received %d", size, tvb:len()))
        return false
    end
    return true
end

local function dissectBeam(tvb, tree, offset)
    local beamTree = tree:add(tvb(offset, 60), "Beam command")
    beamTree:add_le(f.beamType, tvb(offset, 1))
    beamTree:add(f.padding, tvb(offset + 1, 3))
    beamTree:add_le(f.beamTimestamp, tvb(offset + 4, 4))
    beamTree:add_le(f.beamId, tvb(offset + 8, 4))
    beamTree:add_le(f.commandCount, tvb(offset + 12, 4))
    addNumericFields(beamTree, tvb, offset + 16, beamFields, 4)
    beamTree:add(f.padding, tvb(offset + 32, 28))
end

local function dissectTrack(tvb, tree, offset, index)
    local trackTree = tree:add(tvb(offset, 88), "Track " .. index)
    trackTree:add_le(f.trackId, tvb(offset, 2))
    trackTree:add_le(f.trackType, tvb(offset + 2, 1))
    trackTree:add_le(f.memoryFlag, tvb(offset + 3, 1))
    addNumericFields(trackTree, tvb, offset + 4, trackFields, 4)
    trackTree:add_le(f.trackTimestamp, tvb(offset + 48, 4))
    trackTree:add_le(f.trackBeamId, tvb(offset + 52, 4))
    trackTree:add_le(f.trackCommandCount, tvb(offset + 56, 4))
    trackTree:add(f.padding, tvb(offset + 60, 28))
end

local function dissectItems(tvb, tree, pinfo, messageId)
    local isPlot = messageId == 0xBA01
    local isDeletion = messageId == 0xAC03
    local offset = isPlot and 80 or 20
    local itemSize = isPlot and 24 or (isDeletion and 2 or 88)
    local countField = isPlot and f.plotCount or (isDeletion and f.deletionCount or f.trackCount)
    if tvb:len() < offset + 4 then
        tree:add_proto_expert_info(malformed, "Missing item count header")
        return
    end
    if isPlot then
        dissectBeam(tvb, tree, HEADER_SIZE)
    end
    local count = tvb(offset, 2):le_uint()
    tree:add_le(countField, tvb(offset, 2))
    tree:add(f.padding, tvb(offset + 2, 2))
    pinfo.cols.info:append(string.format(" (%d items)", count))
    if count > MAXIMUM_ITEM_COUNT then
        tree:add_proto_expert_info(malformed, "Item count exceeds 100")
        return
    end
    offset = offset + 4
    -- ST_DelTrkMsg is a fixed 224-byte declaration. Also accept a compact
    -- count-sized array for analysis; the application has no deletion encoder.
    if not (isDeletion and tvb:len() == 224) and
        not requireSize(tree, tvb, offset + count * itemSize) then
        return
    end
    for index = 1, count do
        if isPlot then
            local plotTree = tree:add(tvb(offset, itemSize), "Plot " .. index)
            plotTree:add_le(f.plotId, tvb(offset, 4))
            addNumericFields(plotTree, tvb, offset + 4, plotFields, 4)
        elseif isDeletion then
            tree:add_le(f.deletedTrackId, tvb(offset, 2))
        else
            dissectTrack(tvb, tree, offset, index)
        end
        offset = offset + itemSize
    end
    if tvb:len() > offset then
        tree:add(f.raw, tvb(offset, tvb:len() - offset)):append_text(" (unused deletion slots)")
    end
end

local function dissectMessage(tvb, pinfo, tree)
    local messageTree = tree:add(aesa, tvb())
    pinfo.cols.protocol = "AESA"
    if tvb:len() < HEADER_SIZE then
        messageTree:add_proto_expert_info(malformed, "Incomplete common header")
        return tvb:len()
    end
    local messageId = tvb(0, 2):le_uint()
    local name = MESSAGE_NAMES[messageId] or string.format("Unknown ID 0x%04X", messageId)
    messageTree:append_text(": " .. name)
    pinfo.cols.info:append(name)
    local headerTree = messageTree:add(tvb(0, HEADER_SIZE), "Common header")
    headerTree:add_le(f.messageId, tvb(0, 2))
    headerTree:add_le(f.version, tvb(2, 1))
    headerTree:add(f.padding, tvb(3, 1))
    headerTree:add_le(f.blockSize, tvb(4, 4))
    headerTree:add_le(f.timeSec, tvb(8, 4))
    headerTree:add_le(f.timeNsec, tvb(12, 4))
    headerTree:add_le(f.sourceId, tvb(16, 1))
    headerTree:add_le(f.destId, tvb(17, 1))
    headerTree:add(f.padding, tvb(18, 2))

    local blockSize = tvb(4, 4):le_uint()
    if blockSize < HEADER_SIZE or blockSize > MAXIMUM_FRAME_SIZE then
        messageTree:add_proto_expert_info(malformed, "block_size must be between 20 and 65536")
        return tvb:len()
    end
    if not requireSize(messageTree, tvb, blockSize) then
        return tvb:len()
    end
    if messageId == 0xCA01 then
        if requireSize(messageTree, tvb, 24) then
            messageTree:add_le(f.operation, tvb(20, 1))
            messageTree:add(f.padding, tvb(21, 3))
        end
    elseif messageId == 0xAB01 then
        if requireSize(messageTree, tvb, 80) then
            dissectBeam(tvb, messageTree, HEADER_SIZE)
        end
    elseif messageId == 0xAC02 or messageId == 0xBA02 then
        if requireSize(messageTree, tvb, 68) then
            addNumericFields(messageTree, tvb, HEADER_SIZE, attitudeFields, 8)
        end
    elseif messageId == 0xBA01 or messageId == 0xAC01 or messageId == 0xAC03 then
        dissectItems(tvb, messageTree, pinfo, messageId)
    else
        messageTree:add_proto_expert_info(unknownMessage)
        if tvb:len() > HEADER_SIZE then
            messageTree:add(f.raw, tvb(HEADER_SIZE, tvb:len() - HEADER_SIZE))
        end
    end
    return tvb:len()
end

local function getMessageLength(tvb, pinfo, offset)
    local blockSize = tvb(offset + 4, 4):le_uint()
    if blockSize < HEADER_SIZE or blockSize > MAXIMUM_FRAME_SIZE then
        -- There is no magic marker for resynchronization. Consume the available
        -- bytes as one malformed message instead of requesting unbounded memory.
        return tvb:len() - offset
    end
    return blockSize
end

function aesa.dissector(tvb, pinfo, tree)
    pinfo.cols.info:clear()
    local function dissectPdu(pdu, packetInfo, packetTree)
        if tostring(packetInfo.cols.info) ~= "" then
            packetInfo.cols.info:append("; ")
        end
        return dissectMessage(pdu, packetInfo, packetTree)
    end
    dissect_tcp_pdus(tvb, tree, HEADER_SIZE, getMessageLength, dissectPdu, true)
    return tvb:len()
end

local tcpPorts = DissectorTable.get("tcp.port")
local registeredPorts = aesa.prefs.tcp_ports
tcpPorts:add(registeredPorts, aesa)
tcpPorts:add_for_decode_as(aesa)

function aesa.prefs_changed()
    tcpPorts:remove(registeredPorts, aesa)
    registeredPorts = aesa.prefs.tcp_ports
    tcpPorts:add(registeredPorts, aesa)
end
