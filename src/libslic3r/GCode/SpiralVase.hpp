#ifndef slic3r_SpiralVase_hpp_
#define slic3r_SpiralVase_hpp_

#include "../libslic3r.h"
#include "../GCodeReader.hpp"

namespace Slic3r {

class SpiralVase
{
public:
    class SpiralPoint
    {
    public:
        SpiralPoint(float paramx, float paramy) : x(paramx), y(paramy) {}

    public:
        float x, y;
    };
    SpiralVase(const PrintConfig &config) : m_config(config)
    {
        m_reader.z() = (float)m_config.z_offset;
        m_reader.apply_config(m_config);
        m_previous_layer = NULL;
        m_smooth_spiral = config.spiral_mode_smooth;
    };
    ~SpiralVase() { delete m_previous_layer; }
    SpiralVase(const SpiralVase&) = delete;
    SpiralVase& operator=(const SpiralVase&) = delete;

    void 		enable(bool en) {
   		m_transition_layer = en && ! m_enabled;
    	m_enabled 		   = en;
    }

    std::string process_layer(const std::string &gcode, bool last_layer);
    // Internal scope emitted around the selected loop. BEGIN carries bottom and
    // top Z independently of travel lifts. Both markers are consumed here.
    static constexpr const char* primary_begin = ";_SPIRAL_VASE_BEGIN";
    static constexpr const char* primary_end = ";_SPIRAL_VASE_END";
    void set_max_xy_smoothing(float max) {
        m_max_xy_smoothing = max;
    }
private:
    std::string process_layer_legacy(const std::string &gcode, bool last_layer);
    std::string process_layer_allow_islands(const std::string &gcode, bool last_layer);

    const PrintConfig  &m_config;
    GCodeReader 		m_reader;
    float               m_max_xy_smoothing = 0.f;

    bool 				m_enabled = false;
    // First spiral vase layer. Layer height has to be ramped up from zero to the target layer height.
    bool 				m_transition_layer = false;
    // Whether to interpolate XY coordinates with the previous layer. Results in no seam at layer changes
    bool                m_smooth_spiral = false;
    std::vector<SpiralPoint> * m_previous_layer;
};
}

#endif // slic3r_SpiralVase_hpp_
