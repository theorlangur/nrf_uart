#ifndef LIB_LD2412_FORMATTERS_H_
#define LIB_LD2412_FORMATTERS_H_

#include "lib_ld2412.hpp"

template<>
struct tools::formatter_t<hlk::LD2412::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "Err\\{uart=[{}] at {} with {} }", e.uartErr, e.pLocation, hlk::LD2412::err_to_str(e.code));
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::CmdErr>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::CmdErr const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "CmdErr\\{{Err=[{}]; return={} }", e.e, e.returnCode);
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::TargetState>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::TargetState const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case hlk::LD2412::TargetState::Clear: pStr = "Clear"; break;
            case hlk::LD2412::TargetState::Still: pStr = "Still"; break;
            case hlk::LD2412::TargetState::Move: pStr = "Move"; break;
            case hlk::LD2412::TargetState::MoveAndStill: pStr = "MoveAndStill"; break;
            case hlk::LD2412::TargetState::BackgroundAnalysisRunning: pStr = "BackCheckRunning"; break;
            case hlk::LD2412::TargetState::BackgroundAnalysisOk: pStr = "BackCheckOk"; break;
            case hlk::LD2412::TargetState::BackgroundAnalysisFailed: pStr = "BackCheckFailed"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::SystemMode>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::SystemMode const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case hlk::LD2412::SystemMode::Simple: pStr = "Simple"; break;
            case hlk::LD2412::SystemMode::Energy: pStr = "Energy"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::LightSensitivity>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::LightSensitivity const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case hlk::LD2412::LightSensitivity::Off: pStr = "Off"; break;
            case hlk::LD2412::LightSensitivity::DetectWhenBiggerThan: pStr = "BiggerThan"; break;
            case hlk::LD2412::LightSensitivity::DetectWhenLessThan: pStr = "LessThan"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::PresenceResult>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::PresenceResult const& p)
    {
        return tools::format_to(std::forward<Dest>(dst), "[{}; move(dist={}cm; energy={}); still(dist={}cm; energy={})]"
                , p.m_State
                , p.m_MoveDistance, p.m_MoveEnergy
                , p.m_StillDistance, p.m_StillEnergy
            );
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::Engeneering>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::Engeneering const& p)
    {
        return tools::format_to(std::forward<Dest>(dst), 
                "Max Move Gate:{} Max Still Gate:{}\n"
                "Move: {}\n"
                "Still: {}\n"
                "Light: {}\n"
                , p.m_MaxMoveGate, p.m_MaxStillGate,
                  p.m_MoveEnergy,
                  p.m_StillEnergy,
                  p.m_Light
            );
    }
};

template<>
struct tools::formatter_t<hlk::LD2412::Version>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, hlk::LD2412::Version const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "v{}.{}.{}" , v.m_Major, v.m_Minor, v.m_Misc);
    }
};

inline bool operator&(hlk::LD2412::TargetState s1, hlk::LD2412::TargetState s2)
{
    return (uint8_t(s1) & uint8_t(s2)) != 0;
}
#endif
