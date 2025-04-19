#pragma once

#include <string>
#include <iostream>
#include <format>
#include <Eigen/Eigen>


template <typename EigenExprTypeT>
concept EigenTypeMatExpr = requires(const EigenExprTypeT t) {
    std::remove_cvref_t<EigenExprTypeT>::RowsAtCompileTime;
    std::remove_cvref_t<EigenExprTypeT>::ColsAtCompileTime;
    typename std::remove_cvref_t<EigenExprTypeT>::Scalar;
    {
        t.size()
    } -> std::same_as<typename Eigen::Index>;
    {
        t.rows()
    } -> std::same_as<typename Eigen::Index>;
    {
        t.cols()
    } -> std::same_as<typename Eigen::Index>;
};

enum class EigenCustomFormats {
    Default,                //
    CleanFormat,            // cf
    HeavyFormat,            // hf
    SingleLineFormat,       // sfl
    HighPrecisionFormat,    // hpf
    DebuggingFormat         // df
};

static const auto defaultFormat = Eigen::IOFormat();
static const auto cleanFormat = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");
static const auto heavyFormat = Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
static const auto singleLineFormat =
    Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
static const auto highPrecisionFormat =
    Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
static const auto debuggingFormat =
    Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "\n", "");


template <EigenTypeMatExpr MatT>
struct std::formatter<MatT> {
        constexpr auto parse(format_parse_context& ctx) {
            const std::string_view fmt(ctx.begin(), ctx.end());
            if (fmt.starts_with("cf"))
                _format = EigenCustomFormats::CleanFormat;
            if (fmt.starts_with("hf"))
                _format = EigenCustomFormats::HeavyFormat;
            if (fmt.starts_with("sfl"))
                _format = EigenCustomFormats::SingleLineFormat;
            if (fmt.starts_with("hpf"))
                _format = EigenCustomFormats::HighPrecisionFormat;
            if (fmt.starts_with("df"))
                _format = EigenCustomFormats::DebuggingFormat;
            return ctx.begin() + fmt.find_first_of('}');

        }

        // Format the type for output
        template <typename FormatContext>
        auto format(const MatT& m, FormatContext& ctx) const {
            switch (_format) {
                case EigenCustomFormats::CleanFormat:         return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(cleanFormat)).str());
                case EigenCustomFormats::HeavyFormat:         return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(heavyFormat)).str());
                case EigenCustomFormats::SingleLineFormat:    return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(singleLineFormat)).str());
                case EigenCustomFormats::HighPrecisionFormat: return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(highPrecisionFormat)).str());
                case EigenCustomFormats::DebuggingFormat:     return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(debuggingFormat)).str());
                default:                                      return std::format_to(ctx.out(), "{}", (std::stringstream{} << std::fixed << m.format(defaultFormat)).str());
            }

        }

    private:
        EigenCustomFormats _format{ EigenCustomFormats::Default };
};

template <EigenTypeMatExpr MatT>
std::ostream& operator<<(std::ostream& os, const MatT& mat)
{
    return os << std::format("{:hf}", mat);
}
