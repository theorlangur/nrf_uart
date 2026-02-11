#include <algorithm>
#include <cstring>
#include <nrf_uart/periphery/lib_ld2412.hpp>

#define DBG_UART Channel::DbgNow _dbg_uart{this}; 
#define DBG_ME DbgNow _dbg_me{this}; 

#define LD2412_TRY_UART_COMM(f, location, ec) \
    if (auto r = f; !r) \
        return to_result(std::move(r), location, ec)

#define LD2412_TRY_UART_COMM_CMD(f, location, ec) \
    if (auto r = f; !r) \
        return to_cmd_result(std::move(r), location, ec)

#define LD2412_TRY_UART_COMM_CMD_WITH_RETRY(f, location, ec) \
    if (auto r = f; !r) \
    {\
        if (retry) \
        {\
            printk("Failed on " #f "\n");\
            continue;\
        }\
        return to_cmd_result(std::move(r), location, ec);\
    }

const char* LD2412::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::Init: return "Init";
        case ErrorCode::SendFrame: return "SendFrame";
        case ErrorCode::SendFrame_Incomplete: return "SendFrame_Incomplete";
        case ErrorCode::SendCommand_InvalidResponse: return "SendCommand_InvalidResponse";
        case ErrorCode::SendCommand_FailedWrite: return "SendCommand_FailedWrite";
        case ErrorCode::SendCommand_FailedRead: return "SendCommand_FailedRead";
        case ErrorCode::SendCommand_WrongFormat: return "SendCommand_WrongFormat";
        case ErrorCode::SendCommand_Failed: return "SendCommand_Failed";
        case ErrorCode::SendCommand_InsufficientSpace: return "SendCommand_InsufficientSpace";
        case ErrorCode::RecvFrame_Malformed: return "RecvFrame_Malformed";
        case ErrorCode::RecvFrame_Incomplete: return "RecvFrame_Incomplete";
        case ErrorCode::SimpleData_Malformed: return "SimpleData_Malformed";
        case ErrorCode::EnergyData_Malformed: return "EnergyData_Malformed";
        case ErrorCode::SimpleData_Failure: return "SimpleData_Failure";
        case ErrorCode::EnergyData_Failure: return "EnergyData_Failure";
        case ErrorCode::FillBuffer_NoSpace: return "FillBuffer_NoSpace";
        case ErrorCode::FillBuffer_ReadFailure: return "FillBuffer_ReadFailure";
        case ErrorCode::MatchError: return "MatchError";
        case ErrorCode::RestartFailed: return "RestartFailed";
        case ErrorCode::FactoryResetFailed: return "FactoryResetFailed";
        case ErrorCode::BTFailed: return "BTFailed";
        case ErrorCode::WrongState: return "WrongState";
    }
    return "unknown";
}

/**********************************************************************/
/* LD2412 templates                                                   */
/**********************************************************************/
template<class E>
LD2412::ExpectedResult LD2412::to_result(E &&e, const char* pLocation, ErrorCode ec)
{
    using PureE = std::remove_cvref_t<E>;
    if constexpr(is_expected_type_v<PureE>)
    {
        if constexpr (std::is_same_v<PureE, ExpectedResult>)
            return std::move(e);
        else
            return to_result(e.error(), pLocation, ec);
    }else if constexpr (std::is_same_v<PureE,::Err>)
        return ExpectedResult(std::unexpected(Err{e, pLocation, ec}));
    else if constexpr (std::is_same_v<PureE,Err>)
        return ExpectedResult(std::unexpected(e));
    else if constexpr (std::is_same_v<PureE,CmdErr>)
        return ExpectedResult(std::unexpected(e.e));
    else
    {
        static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
        return ExpectedResult(std::unexpected(Err{}));
    }
}

template<class E>
LD2412::ExpectedGenericCmdResult LD2412::to_cmd_result(E &&e, const char* pLocation, ErrorCode ec)
{
    using PureE = std::remove_cvref_t<E>;
    if constexpr(is_expected_type_v<PureE>)
    {
        if constexpr (std::is_same_v<PureE, ExpectedGenericCmdResult>)
            return std::move(e);
        else
            return to_cmd_result(e.error(), pLocation, ec);
    }else if constexpr (std::is_same_v<PureE,::Err>)
        return ExpectedGenericCmdResult(std::unexpected(CmdErr{Err{e, pLocation, ec}, 0}));
    else if constexpr (std::is_same_v<PureE,Err>)
        return ExpectedGenericCmdResult(std::unexpected(CmdErr{e, 0}));
    else if constexpr (std::is_same_v<PureE,CmdErr>)
        return ExpectedGenericCmdResult(std::unexpected(e));
    else
    {
        static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
        return ExpectedGenericCmdResult(std::unexpected(CmdErr{}));
    }
}


template<class...T>
LD2412::ExpectedResult LD2412::SendFrame(T&&... args)
{
    using namespace uart::primitives;
    //1. header
    LD2412_TRY_UART_COMM(Send(kFrameHeader, sizeof(kFrameHeader)), "SendFrameV2", ErrorCode::SendFrame);

    //2. length
    {
        uint16_t len = (sizeof(args) + ...);
        LD2412_TRY_UART_COMM(Send((uint8_t const*)&len, sizeof(len)), "SendFrameV2", ErrorCode::SendFrame);
    }
    //3. data
    LD2412_TRY_UART_COMM(uart::primitives::write_any(*this, std::forward<T>(args)...), "SendFrameV2", ErrorCode::SendFrame);

    //4. footer
    LD2412_TRY_UART_COMM(Send(kFrameFooter, sizeof(kFrameFooter)), "SendFrameV2", ErrorCode::SendFrame);

    return std::ref(*this);
}

template<class...T>
LD2412::ExpectedResult LD2412::RecvFrame(T&&... args)
{
    constexpr const size_t arg_size = (uart::primitives::uart_sizeof<std::remove_cvref_t<T>>() + ...);
    LD2412_TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameHeader), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    if (m_dbg) printk("RecvFrameV2: matched header\n"); 
    uint16_t len;
    LD2412_TRY_UART_COMM(uart::primitives::read_into(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    if (m_dbg) printk("RecvFrameV2: len: %d\n", len);
    if (arg_size > len)
        return std::unexpected(Err{{}, "RecvFrameV2 len invalid", ErrorCode::RecvFrame_Malformed}); 

    LD2412_TRY_UART_COMM(uart::primitives::read_any_limited(*this, len, std::forward<T>(args)...), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    if (len)
    {
        LD2412_TRY_UART_COMM(uart::primitives::skip_bytes(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    }
    if (m_dbg) printk("RecvFrameV2: mathcing footer\n");
    LD2412_TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameFooter), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
    if (m_dbg) printk("RecvFrameV2: matched footer\n");
    return std::ref(*this);
}

template<class CmdT, class... ToSend, class... ToRecv>
LD2412::ExpectedGenericCmdResult LD2412::SendCommand(CmdT cmd, std::tuple<ToSend...> sendArgs, std::tuple<ToRecv...> recvArgs)
{
    static_assert(sizeof(CmdT) == 2, "must be 2 bytes");
    if (GetDefaultWait() < kDefaultWait)
        SetDefaultWait(kDefaultWait);
    if (m_dbg)
        printk("SendCommandV2 %x\n", (int)cmd);
    uint16_t status;
    auto SendFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){
        return SendFrame(cmd, std::get<idx>(sendArgs)...);
    };
    auto RecvFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){ 
        return RecvFrame(
            uart::primitives::match_t{uint16_t(cmd | 0x100)}, 
            status, 
            uart::primitives::callback_t{[&]()->Channel::ExpectedResult{
                if (m_dbg) printk("Recv frame resp. Status %d\n", status);
                if (status != 0)
                    return std::unexpected(::Err{"SendCommandV2 status", status});
                return std::ref((Channel&)*this);
            }},
            std::get<idx>(recvArgs)...);
    };

    constexpr int kMaxRetry = 1;
    for(int retry=kMaxRetry; retry >= 0; --retry)
    {
        if (retry != kMaxRetry)
        {
            if (m_dbg) printk("Sending command %x retry: %d\n", uint16_t(cmd), (kMaxRetry - retry));
            k_msleep(kDefaultWait); 
            (void)Channel::Drain(false).has_value();
        }
        LD2412_TRY_UART_COMM_CMD_WITH_RETRY(SendFrameExpandArgs(std::make_index_sequence<sizeof...(ToSend)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
        if (m_dbg) printk("Wait all\n");
        LD2412_TRY_UART_COMM_CMD_WITH_RETRY(WaitAllSent(), "SendCommandV2", ErrorCode::SendCommand_Failed);
        if (m_dbg) printk("Receiving %d args\n", sizeof...(ToRecv));
        LD2412_TRY_UART_COMM_CMD_WITH_RETRY(RecvFrameExpandArgs(std::make_index_sequence<sizeof...(ToRecv)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
        break;
    }
    return std::ref(*this);
}

/**********************************************************************/
/* LD2412                                                             */
/**********************************************************************/
LD2412::LD2412(const struct device *pUART):
    uart::Channel(pUART)
{
}

LD2412::ExpectedResult LD2412::Init()
{
    SetDefaultWait(kDefaultWait);
    LD2412_TRY_UART_COMM(Configure(), "Init", ErrorCode::Init);
    LD2412_TRY_UART_COMM(Open(), "Init", ErrorCode::Init);
    return ReloadConfig();
}

LD2412::ExpectedResult LD2412::ReloadConfig()
{
    RxBlock _RxBlock(*this);
    LD2412_TRY_UART_COMM(OpenCommandMode(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(UpdateVersion(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::ReadBaseParams, to_send(), to_recv(m_Configuration.m_Base)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetMoveSensitivity, to_send(), to_recv(m_Configuration.m_MoveThreshold)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetStillSensitivity, to_send(), to_recv(m_Configuration.m_StillThreshold)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetMAC, to_send(uint16_t(0x0001)), to_recv(m_BluetoothMAC)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetDistanceRes, to_send(), to_recv(m_DistanceResolution)), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetLightSensitivity, to_send(), to_recv(m_Configuration.m_LightSense)), "ReloadConfig: LightSense", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(CloseCommandMode(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::UpdateDistanceRes()
{
    LD2412_TRY_UART_COMM(OpenCommandMode(), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::GetDistanceRes, to_send(), to_recv(m_DistanceResolution)), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(CloseCommandMode(), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::SwitchBluetooth(bool on)
{
    namespace uartp = uart::primitives;
    SetDefaultWait(kDefaultWait);
    LD2412_TRY_UART_COMM(OpenCommandMode(), "SwitchBluetooth", ErrorCode::BTFailed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::SwitchBluetooth, to_send(uint16_t(on)), to_recv()), "SwitchBluetooth", ErrorCode::BTFailed);
    LD2412_TRY_UART_COMM(SendFrame(Cmd::Restart), "SwitchBluetooth", ErrorCode::BTFailed);
    k_msleep(1000); 
    LD2412_TRY_UART_COMM(uartp::flush_and_wait(*this, {kRestartTimeout, "SwitchBluetooth"}), "SwitchBluetooth", ErrorCode::BTFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        LD2412_TRY_UART_COMM(rs, "SwitchBluetooth", ErrorCode::BTFailed);
    }
    return ReloadConfig();
}

LD2412::ExpectedResult LD2412::Restart()
{
    namespace uartp = uart::primitives;
    SetDefaultWait(kDefaultWait);
    LD2412_TRY_UART_COMM(OpenCommandMode(), "Restart", ErrorCode::RestartFailed);
    LD2412_TRY_UART_COMM(SendFrame(Cmd::Restart), "Restart", ErrorCode::RestartFailed);
    k_msleep(1000); 
    LD2412_TRY_UART_COMM(uartp::flush_and_wait(*this, {kRestartTimeout, "Restart"}), "Restart", ErrorCode::RestartFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        LD2412_TRY_UART_COMM(rs, "SwitchBluetooth", ErrorCode::BTFailed);
    }
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::FactoryReset()
{
    namespace uartp = uart::primitives;
    SetDefaultWait(uart::duration_ms_t(1000));
    LD2412_TRY_UART_COMM(OpenCommandMode(), "FactoryReset", ErrorCode::FactoryResetFailed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::FactoryReset, to_send(), to_recv()), "FactoryReset", ErrorCode::FactoryResetFailed);
    LD2412_TRY_UART_COMM(SendFrame(Cmd::Restart), "FactoryReset", ErrorCode::FactoryResetFailed);
    k_msleep(1000); 
    LD2412_TRY_UART_COMM(uartp::flush_and_wait(*this, {kRestartTimeout, "FactoryReset"}), "FactoryReset", ErrorCode::FactoryResetFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        LD2412_TRY_UART_COMM(rs, "FactoryReset", ErrorCode::FactoryResetFailed);
    }
    return ReloadConfig();
}

LD2412::ExpectedOpenCmdModeResult LD2412::OpenCommandMode()
{
    uint16_t protocol_version = 1;
    OpenCmdModeResponse r;
    if (auto r = SendFrame(Cmd::OpenCmd, protocol_version); !r)
        return std::unexpected(CmdErr{r.error(), 0});
    k_msleep(100); 

    if (auto rs = SendCommand(Cmd::OpenCmd, to_send(protocol_version), to_recv(r.protocol_version, r.buffer_size)); !rs)
        return std::unexpected(rs.error());

    (void)Channel::Drain(false).has_value();
    return OpenCmdModeRetVal{std::ref(*this), r};
}

LD2412::ExpectedGenericCmdResult LD2412::CloseCommandMode()
{
    return SendCommand(Cmd::CloseCmd, to_send(), to_recv());
}

LD2412::ExpectedGenericCmdResult LD2412::SetSystemModeInternal(SystemMode mode)
{
    Cmd c = mode == SystemMode::Energy ? Cmd::EnterEngMode : Cmd::LeaveEngMode;
    return SendCommand(c, to_send(), to_recv());
}

LD2412::ExpectedGenericCmdResult LD2412::SetDistanceResInternal(DistanceRes r)
{
    DistanceResBuf resBuf{r};
    return SendCommand(Cmd::SetDistanceRes, to_send(resBuf), to_recv());
}

LD2412::ExpectedGenericCmdResult LD2412::UpdateVersion()
{
    namespace uartp = uart::primitives;
    constexpr uint16_t kVersionBegin = 0x2412;
    return SendCommand(Cmd::ReadVer, to_send(), to_recv(uartp::match_t{kVersionBegin}, m_Version));
}

LD2412::ExpectedResult LD2412::ReadFrame()
{
//ReadFrame: Read bytes: f4 f3 f2 f1 0b 00 02 aa 02 00 00 00 a0 00 64 55 00 f8 f7 f6 f5 
    constexpr uint8_t report_begin[] = {0xaa};
    constexpr uint8_t report_end[] = {0x55};
    namespace uartp = uart::primitives;
    SystemMode mode;
    uint8_t check;
    uint16_t reportLen = 0;
    LD2412_TRY_UART_COMM(uartp::read_until(*this, kDataFrameHeader[0], uart::duration_ms_t(1000), "Searching for header"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    LD2412_TRY_UART_COMM(uartp::match_bytes(*this, kDataFrameHeader, "Matching header"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    LD2412_TRY_UART_COMM(uartp::read_any(*this, reportLen, mode), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    LD2412_TRY_UART_COMM(uartp::match_bytes(*this, report_begin, "Matching rep begin"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    LD2412_TRY_UART_COMM(uartp::read_into(*this, m_Presence), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);//simple Part of the detection is always there
    if (mode == SystemMode::Energy)
    {
        if ((reportLen - 4 - sizeof(m_Presence)) != sizeof(m_Engeneering))
            return std::unexpected(Err{{"Wrong engeneering size"}, "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed});
        LD2412_TRY_UART_COMM(uartp::read_into(*this, m_Engeneering), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    }
    LD2412_TRY_UART_COMM(uartp::match_bytes(*this, report_end, "Matching rep end"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    LD2412_TRY_UART_COMM(uartp::read_into(*this, check), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);//simple Part of the detection is always there
    LD2412_TRY_UART_COMM(uartp::match_bytes(*this, kDataFrameFooter, "Matching footer"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::TryReadSingleFrame(int attempts, bool flush, Drain drain)
{
    if (m_ContinuousRead)
        return TryReadFrame(attempts, flush, drain);
    RxBlock _RxBlock(*this);
    return TryReadFrame(attempts, flush, drain);
}

void LD2412::StartContinuousReading()
{
    m_ContinuousRead = true;
    AllowReadUpTo(m_recvBuf, sizeof(m_recvBuf));
}

void LD2412::StopContinuousReading()
{
    StopReading();
    m_ContinuousRead = false;
}

LD2412::ExpectedResult LD2412::TryReadFrame(int attempts, bool flush, Drain drain)
{
    if (drain != Drain::No)
    {
        SetDefaultWait(uart::duration_ms_t(0));
        int i = 0;
        for(; i < 100; ++i)
        {
            if (auto r = ReadFrame(); !r)
            {
                if (i > 0)//if i is at least 2 that means that at least 1 iteration was successful 
                    break;
                else if (drain == Drain::Try)
                    return TryReadFrame(attempts, flush, Drain::No);
                else
                    return r;
            }
        }
    }else
    {
        SetDefaultWait(uart::duration_ms_t(kDefaultWait));
        auto ec = (m_Mode == SystemMode::Energy) ? ErrorCode::EnergyData_Failure : ErrorCode::SimpleData_Failure;
        for(int i = 0; i < attempts; ++i)
        {
            if (auto r = ReadFrame(); !r)
            {
                if ((i + 1) == attempts)
                    return to_result(std::move(r), "LD2412::TryReadFrame", ec);
            }else
                return std::ref(*this);
        }
    }
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::RunDynamicBackgroundAnalysis()
{
    if (m_DynamicBackgroundAnalysis)
        return std::unexpected(Err{{}, "RunDynamicBackgroundAnalysis", ErrorCode::WrongState});
    SetDefaultWait(kDefaultWait);
    LD2412_TRY_UART_COMM(OpenCommandMode(), "RunDynamicBackgroundAnalysis", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::RunDynamicBackgroundAnalysis, to_send(), to_recv()), "RunDynamicBackgroundAnalysis", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(CloseCommandMode(), "RunDynamicBackgroundAnalysis", ErrorCode::SendCommand_Failed);
    m_DynamicBackgroundAnalysis = true;
    return std::ref(*this);
}

bool LD2412::IsDynamicBackgroundAnalysisRunning()
{
    if (m_DynamicBackgroundAnalysis)
        (void)QueryDynamicBackgroundAnalysisRunState().has_value();
    return m_DynamicBackgroundAnalysis;
}

LD2412::ExpectedResult LD2412::QueryDynamicBackgroundAnalysisRunState()
{
    SetDefaultWait(kDefaultWait);
    uint16_t active = 0;
    LD2412_TRY_UART_COMM(OpenCommandMode(), "QueryDynamicBackgroundAnalysisRunState", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(SendCommand(Cmd::QuearyDynamicBackgroundAnalysis, to_send(), to_recv(active)), "QueryDynamicBackgroundAnalysisRunState", ErrorCode::SendCommand_Failed);
    LD2412_TRY_UART_COMM(CloseCommandMode(), "QueryDynamicBackgroundAnalysisRunState", ErrorCode::SendCommand_Failed);
    m_DynamicBackgroundAnalysis = active != 0;
    return std::ref(*this);
}

/**********************************************************************/
/* ConfigBlock                                                        */
/**********************************************************************/
LD2412::ConfigBlock& LD2412::ConfigBlock::SetSystemMode(SystemMode mode)
{
    m_Changed.Mode = true;
    m_NewMode = mode;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetDistanceRes(DistanceRes r)
{
    m_Changed.DistanceRes = true;
    m_NewDistanceRes = r;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMinDistance(int dist)
{
    m_Changed.MinDistance = true;
    m_Configuration.m_Base.m_MinDistanceGate = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMinDistanceRaw(uint8_t dist)
{
    m_Changed.MinDistance = true;
    m_Configuration.m_Base.m_MinDistanceGate = std::clamp(dist, uint8_t(1), uint8_t(12));
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistance(int dist)
{
    m_Changed.MaxDistance = true;
    m_Configuration.m_Base.m_MaxDistanceGate = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistanceRaw(uint8_t dist)
{
    m_Changed.MaxDistance = true;
    m_Configuration.m_Base.m_MaxDistanceGate = std::clamp(dist, uint8_t(1), uint8_t(12));
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetTimeout(uint16_t t)
{
    m_Changed.Timeout = true;
    m_Configuration.m_Base.m_Duration = t;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetOutPinPolarity(bool lowOnPresence)
{
    m_Changed.OutPin = true;
    m_Configuration.m_Base.m_OutputPinPolarity = lowOnPresence;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMoveThreshold(uint8_t gate, uint8_t energy)
{
    if (gate > 13)
        return *this;

    m_Changed.MoveThreshold = true;
    m_Configuration.m_MoveThreshold[gate] = energy;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetStillThreshold(uint8_t gate, uint8_t energy)
{
    if (gate > 13)
        return *this;

    m_Changed.StillThreshold = true;
    m_Configuration.m_StillThreshold[gate] = energy;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetLightSensitivity(LightSensitivity senseMode, uint8_t lightThreshold)
{
    m_Changed.LightSens = true;
    m_Configuration.m_LightSense.m_Mode = senseMode;
    m_Configuration.m_LightSense.m_ThresholdLevel = lightThreshold;
    return *this;
}

LD2412::ExpectedResult LD2412::ConfigBlock::EndChange()
{
    if (!m_Changes)
        return std::ref(d);
    ScopeExit clearChanges = [&]{ m_Changes = 0; };
    if (d.m_ContinuousRead)
        return std::unexpected(Err{{}, "ConfigBlock::EndChange", ErrorCode::WrongState});

    LD2412_TRY_UART_COMM(d.OpenCommandMode(), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    if (m_Changed.Mode)
    {
        d.m_Mode = m_NewMode; 
        LD2412_TRY_UART_COMM(d.SetSystemModeInternal(d.m_Mode), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.DistanceRes)
    {
        d.m_DistanceResolution.m_Res = m_NewDistanceRes; 
        LD2412_TRY_UART_COMM(d.SetDistanceResInternal(m_NewDistanceRes), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.MinDistance || m_Changed.MaxDistance || m_Changed.Timeout || m_Changed.OutPin)
    {
        d.m_Configuration.m_Base = m_Configuration.m_Base;
        LD2412_TRY_UART_COMM(d.SendCommand(Cmd::WriteBaseParams ,to_send(d.m_Configuration.m_Base) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.MoveThreshold)
    {
        std::ranges::copy(m_Configuration.m_MoveThreshold, d.m_Configuration.m_MoveThreshold);
        LD2412_TRY_UART_COMM(d.SendCommand(Cmd::SetMoveSensitivity ,to_send(d.m_Configuration.m_MoveThreshold) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.StillThreshold)
    {
        std::ranges::copy(m_Configuration.m_StillThreshold, d.m_Configuration.m_StillThreshold);
        LD2412_TRY_UART_COMM(d.SendCommand(Cmd::SetStillSensitivity ,to_send(d.m_Configuration.m_StillThreshold) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.LightSens)
    {
        d.m_Configuration.m_LightSense = m_Configuration.m_LightSense;
        LD2412_TRY_UART_COMM(d.SendCommand(Cmd::SetLightSensitivity ,to_send(d.m_Configuration.m_LightSense) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    LD2412_TRY_UART_COMM(d.CloseCommandMode(), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    return std::ref(d);
}
