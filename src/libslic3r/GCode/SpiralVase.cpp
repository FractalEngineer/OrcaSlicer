#include "SpiralVase.hpp"
#include "GCode.hpp"
#include <sstream>
#include <cmath>
#include <cstdio>
#include <limits>
#include <locale>

namespace Slic3r {

namespace SpiralVaseHelpers {
/** == Smooth Spiral Helpers == */
/** Distance between a and b */
float distance(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) { return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)); }

SpiralVase::SpiralPoint subtract(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b)
{
    return SpiralVase::SpiralPoint(a.x - b.x, a.y - b.y);
}

SpiralVase::SpiralPoint add(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) { return SpiralVase::SpiralPoint(a.x + b.x, a.y + b.y); }

SpiralVase::SpiralPoint scale(SpiralVase::SpiralPoint a, float factor) { return SpiralVase::SpiralPoint(a.x * factor, a.y * factor); }

/** dot product */
float dot(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) { return a.x * b.x + a.y * b.y; }

/** Find the point on line ab closes to point c */
SpiralVase::SpiralPoint nearest_point_on_line(SpiralVase::SpiralPoint c, SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b, float& dist)
{
    SpiralVase::SpiralPoint ab      = subtract(b, a);
    SpiralVase::SpiralPoint ca      = subtract(c, a);
    float                   t       = dot(ca, ab) / dot(ab, ab);
    t                               = t > 1 ? 1 : t;
    t                               = t < 0 ? 0 : t;
    SpiralVase::SpiralPoint closest = SpiralVase::SpiralPoint(add(a, scale(ab, t)));
    dist                            = distance(c, closest);
    return closest;
}

/** Given a set of lines defined by points such as line[n] is the line from points[n] to points[n+1],
 *  find the closest point to p that falls on any of the lines */
SpiralVase::SpiralPoint nearest_point_on_lines(SpiralVase::SpiralPoint               p,
                                               std::vector<SpiralVase::SpiralPoint>* points,
                                               bool&                                 found,
                                               float&                                dist)
{
    if (points->size() < 2) {
        found = false;
        return SpiralVase::SpiralPoint(0, 0);
    }
    float                   min = std::numeric_limits<float>::max();
    SpiralVase::SpiralPoint closest(0, 0);
    for (unsigned long i = 0; i < points->size() - 1; i++) {
        float                   currentDist = 0;
        SpiralVase::SpiralPoint current     = nearest_point_on_line(p, points->at(i), points->at(i + 1), currentDist);
        if (currentDist < min) {
            min     = currentDist;
            closest = current;
            found   = true;
        }
    }
    dist = min;
    return closest;
}
} // namespace SpiralVase

std::string SpiralVase::process_layer(const std::string &gcode, bool last_layer)
{
    // Keep the established single-loop processing byte-for-byte unchanged unless
    // disconnected contours were explicitly requested.
    return m_config.spiral_mode_allow_islands ? process_layer_allow_islands(gcode, last_layer)
                                              : process_layer_legacy(gcode, last_layer);
}

std::string SpiralVase::process_layer_legacy(const std::string &gcode, bool last_layer)
{
    /*  This post-processor relies on several assumptions:
        - all layers are processed through it, including those that are not supposed
          to be transformed, in order to update the reader with the XY positions
        - each call to this method includes a full layer, with a single Z move
          at the beginning
        - each layer is composed by suitable geometry (i.e. a single complete loop)
        - loops were not clipped before calling this method  */
    
    // If we're not going to modify G-code, just feed it to the reader
    // in order to update positions.
    if (! m_enabled) {
        m_reader.parse_buffer(gcode);
        return gcode;
    }
    
    // Get total XY length for this layer by summing all extrusion moves.
    float total_layer_length = 0;
    float layer_height = 0;
    float z = 0.f;
    
    {
        //FIXME Performance warning: This copies the GCodeConfig of the reader.
        GCodeReader r = m_reader;  // clone
        bool set_z = false;
        r.parse_buffer(gcode, [&total_layer_length, &layer_height, &z, &set_z]
            (GCodeReader &reader, const GCodeReader::GCodeLine &line) {
            if (line.cmd_is("G1")) {
                if (line.extruding(reader)) {
                    total_layer_length += line.dist_XY(reader);
                } else if (line.has(Z)) {
                    layer_height += line.dist_Z(reader);
                    if (!set_z) {
                        z = line.new_Z(reader);
                        set_z = true;
                    }
                }
            }
        });
    }

    // Remove layer height from initial Z.
    z -= layer_height;

    std::vector<SpiralVase::SpiralPoint>* current_layer = new std::vector<SpiralVase::SpiralPoint>();
    std::vector<SpiralVase::SpiralPoint>* previous_layer = m_previous_layer;

    bool smooth_spiral = m_smooth_spiral;
    std::string new_gcode;
    std::string transition_gcode;
    float max_xy_dist_for_smoothing = m_max_xy_smoothing;
    //FIXME Tapering of the transition layer only works reliably with relative extruder distances.
    // For absolute extruder distances it will be switched off.
    // Tapering the absolute extruder distances requires to process every extrusion value after the first transition
    // layer.
    bool  transition_in = m_transition_layer && m_config.use_relative_e_distances.value;
    bool  transition_out = last_layer && m_config.use_relative_e_distances.value;

    float starting_flowrate  = float(m_config.spiral_starting_flow_ratio.value);
    float finishing_flowrate = float(m_config.spiral_finishing_flow_ratio.value);
    const float min_segment_length = std::max(float(EPSILON), 2 * float(m_config.resolution.value));

    float len = 0.f;
    SpiralVase::SpiralPoint last_point = previous_layer != NULL && previous_layer->size() >0? previous_layer->at(previous_layer->size()-1): SpiralVase::SpiralPoint(0,0);
    m_reader.parse_buffer(gcode, [&new_gcode, &z, total_layer_length, layer_height, transition_in, &len, &current_layer, &previous_layer, &transition_gcode, transition_out, smooth_spiral, &max_xy_dist_for_smoothing, &last_point, starting_flowrate, finishing_flowrate, min_segment_length]
        (GCodeReader &reader, GCodeReader::GCodeLine line) {
        if (line.cmd_is("G1")) {
            // Orca: Filter out retractions at layer change
            if (line.retracting(reader) || (line.extruding(reader) && line.dist_XY(reader) < min_segment_length)) return;
            if (line.has_z() && !(line.has_x() || line.has_y())) {
                // If this is the initial Z move of the layer, replace it with a
                // (redundant) move to the last Z of previous layer.
                line.set(Z, z);
                new_gcode += line.raw() + '\n';
                return;
            } else {
                float dist_XY = line.dist_XY(reader);
                if (line.has_x() || line.has_y()) { // Sometimes lines have X/Y but the move is to the last position
                    if (dist_XY > 0 && line.extruding(reader)) { // Exclude wipe and retract
                        len += dist_XY;
                        float factor = len / total_layer_length;
                        if (transition_in){
                            // Transition layer, interpolate the amount of extrusion starting from spiral_vase_starting_flow_rate to 100%.
                            float starting_e_factor = starting_flowrate + (factor * (1.f - starting_flowrate));
                            line.set(E, line.e() * starting_e_factor, 5 /*decimal_digits*/);
                        } else if (transition_out) {
                            // We want the last layer to ramp down extrusion, but without changing z height!
                            // So clone the line before we mess with its Z and duplicate it into a new layer that ramps down E
                            // We add this new layer at the very end
                            // As with transition_in, the amount is ramped down from 100% to spiral_vase_finishing_flow_rate
                            GCodeReader::GCodeLine transitionLine(line);
                            float finishing_e_factor = finishing_flowrate + ((1.f -factor) * (1.f - finishing_flowrate));
                            transitionLine.set(E, line.e() * finishing_e_factor, 5 /*decimal_digits*/);
                            transition_gcode += transitionLine.raw() + '\n';
                        }
                        // This line is the core of Spiral Vase mode, ramp up the Z smoothly
                        line.set(Z, z + factor * layer_height);
                        if (smooth_spiral) {
                            // Now we also need to try to interpolate X and Y
                            SpiralVase::SpiralPoint p(line.x(), line.y()); // Get current x/y coordinates
                            current_layer->push_back(p);       // Store that point for later use on the next layer
                            if (previous_layer != NULL) {
                                bool        found    = false;
                                float       dist     = 0;
                                SpiralVase::SpiralPoint nearestp = SpiralVaseHelpers::nearest_point_on_lines(p, previous_layer, found, dist);
                                if (found && dist < max_xy_dist_for_smoothing) {
                                    // Interpolate between the point on this layer and the point on the previous layer
                                    SpiralVase::SpiralPoint target = SpiralVaseHelpers::add(SpiralVaseHelpers::scale(nearestp, 1 - factor), SpiralVaseHelpers::scale(p, factor));

                                    // Remove tiny movement
                                    // We need to figure out the distance of this new line!
                                    float modified_dist_XY = SpiralVaseHelpers::distance(last_point, target);
                                    if (modified_dist_XY < min_segment_length)
                                        line.clear();
                                    else {
                                        line.set(X, target.x);
                                        line.set(Y, target.y);
                                        // Scale the extrusion amount according to change in length
                                        line.set(E, line.e() * modified_dist_XY / dist_XY, 5 /*decimal_digits*/);
                                        last_point = target;
                                    }
                                } else {
                                    last_point = p;
                                }
                            }
                        }
                        new_gcode += line.raw() + '\n';
                    }
                    return;
                    /*  Skip travel moves: the move to first perimeter point will
                        cause a visible seam when loops are not aligned in XY; by skipping
                        it we blend the first loop move in the XY plane (although the smoothness
                        of such blend depend on how long the first segment is; maybe we should
                        enforce some minimum length?).
                        When smooth_spiral is enabled, we're gonna end up exactly where the next layer should
                        start anyway, so we don't need the travel move */
                }
            }
        }
        new_gcode += line.raw() + '\n';
        if(transition_out) {
            transition_gcode += line.raw() + '\n';
        }
    });

    delete m_previous_layer;
    m_previous_layer = current_layer;
    
    return new_gcode + transition_gcode;
}

std::string SpiralVase::process_layer_allow_islands(const std::string &gcode, bool last_layer)
{
    float bottom_z = 0.f;
    float top_z = 0.f;
    float total_length = 0.f;
    size_t begin_count = 0;
    size_t end_count = 0;
    bool inside = false;
    bool started = false;
    bool valid = true;
    bool prefix_travel = false;
    SpiralPoint start(0.f, 0.f);
    SpiralPoint end(0.f, 0.f);
    std::string unmarked;

    // Scope is supplied by the loop emitter. Never infer identity from travel,
    // role comments, flow changes, or the size/order of raw extrusion runs.
    GCodeReader scan = m_reader;
    scan.parse_buffer(gcode, [&](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
        if (line.raw().compare(0, strlen(primary_begin), primary_begin) == 0) {
            ++begin_count;
            inside = true;
            std::istringstream heights(line.raw().substr(strlen(primary_begin)));
            heights.imbue(std::locale::classic());
            valid &= bool(heights >> bottom_z >> top_z);
            return;
        }
        if (line.raw() == primary_end) {
            ++end_count;
            valid &= inside && started;
            inside = false;
            return;
        }
        unmarked += line.raw() + '\n';
        if (line.cmd_is("G1") || line.cmd_is("G0")) {
            const float distance = line.dist_XY(reader);
            if (inside && line.extruding(reader) && distance > 0.f) {
                if (!started) {
                    start = SpiralPoint(reader.x(), reader.y());
                    started = true;
                }
                total_length += distance;
                end = SpiralPoint(line.new_X(reader), line.new_Y(reader));
            } else if (!started) {
                prefix_travel |= distance > float(EPSILON);
            } else if (inside) {
                // A disconnected or retracted path cannot be a continuous helix.
                // Leave such a layer conventional instead of dropping any motion.
                valid &= distance <= float(EPSILON) && !line.retracting(reader) && !line.has_z();
            }
        }
    });

    valid &= begin_count == 1 && end_count == 1 && !inside && started &&
             total_length > float(EPSILON) && top_z > bottom_z &&
             SpiralVaseHelpers::distance(start, end) < 0.003f;
    if (!m_enabled || !valid) {
        m_reader = std::move(scan);
        delete m_previous_layer;
        m_previous_layer = nullptr;
        // In particular, never feed a multi-contour layer to the legacy path:
        // its deliberate travel suppression would join unrelated extrusions.
        return begin_count == 0 && end_count == 0 ? gcode : unmarked;
    }

    const bool relative_e = m_config.use_relative_e_distances.value;
    const bool transition_in = m_transition_layer || m_previous_layer == nullptr;
    const bool transition_out = last_layer;
    const bool continuous = !prefix_travel && std::abs(m_reader.z() - bottom_z) < 0.001f;
    const float min_segment_length = std::max(float(EPSILON), 2 * float(m_config.resolution.value));
    float length = 0.f;
    float e_offset = 0.f;
    float finishing_e = 0.f;
    std::vector<SpiralPoint> current_layer{start};
    SpiralPoint last_point = start;
    std::string output;
    std::string finishing;
    inside = false;
    started = false;

    m_reader.parse_buffer(gcode, [&](GCodeReader &reader, GCodeReader::GCodeLine line) {
        if (line.raw().compare(0, strlen(primary_begin), primary_begin) == 0) {
            inside = true;
            return;
        }
        if (line.raw() == primary_end) {
            if (transition_out) {
                if (!relative_e)
                    output += "G92 E0\n";
                output += finishing;
            }
            // Restore the writer's absolute E coordinate after changing flow or
            // replaying the finishing loop, so islands and later layers match it.
            if (!relative_e && (e_offset != 0.f || transition_out)) {
                char reset[64];
                snprintf(reset, sizeof(reset), "G92 E%.5f\n", reader.e());
                output += reset;
            }
            inside = false;
            return;
        }
        if (inside && line.cmd_is("G1") && line.extruding(reader) && line.dist_XY(reader) > 0.f) {
            if (!started) {
                // Travel/retraction/lift commands have already positioned the
                // nozzle over the primary. Descend there, never over an island.
                if (!continuous) {
                    char move[64];
                    snprintf(move, sizeof(move), "G1 Z%.3f\n", bottom_z);
                    output += move;
                }
                started = true;
            }
            const float distance = line.dist_XY(reader);
            const float original_e = line.dist_E(reader);
            length += distance;
            const float factor = std::min(1.f, length / total_length);
            float extrusion = original_e;
            const SpiralPoint point(line.new_X(reader), line.new_Y(reader));
            current_layer.push_back(point);

            if (transition_out) {
                GCodeReader::GCodeLine finish(line);
                const float flow = float(m_config.spiral_finishing_flow_ratio.value);
                const float delta = original_e * (flow + (1.f - factor) * (1.f - flow));
                finishing_e += delta;
                finish.set(E, relative_e ? delta : finishing_e, 5);
                finish.set(Z, top_z);
                std::string move = finish.raw();
                if (!finish.has_f()) {
                    // Keep F after the coordinates: cooling's duplicate-feedrate
                    // removal can strip G1 from a line beginning with G1 F.
                    const size_t comment = move.find(';');
                    move.insert(comment == std::string::npos ? move.size() : comment,
                                " F" + std::to_string(line.new_F(reader)) + " ");
                }
                finishing += move + '\n';
            }
            if (transition_in) {
                const float flow = float(m_config.spiral_starting_flow_ratio.value);
                extrusion *= flow + factor * (1.f - flow);
            }
            if (m_smooth_spiral && m_previous_layer != nullptr) {
                bool found = false;
                float distance_to_previous = 0.f;
                const SpiralPoint nearest = SpiralVaseHelpers::nearest_point_on_lines(
                    point, m_previous_layer, found, distance_to_previous);
                if (found && distance_to_previous < m_max_xy_smoothing) {
                    const SpiralPoint target = SpiralVaseHelpers::add(
                        SpiralVaseHelpers::scale(nearest, 1.f - factor), SpiralVaseHelpers::scale(point, factor));
                    const float modified_distance = SpiralVaseHelpers::distance(last_point, target);
                    // Keep tiny segments instead of deleting material or the
                    // endpoint that brings the helix to the completed layer Z.
                    if (modified_distance >= min_segment_length) {
                        line.set(X, target.x);
                        line.set(Y, target.y);
                        extrusion *= modified_distance / distance;
                    }
                }
            }
            last_point = SpiralPoint(line.new_X(reader), line.new_Y(reader));
            line.set(Z, bottom_z + factor * (top_z - bottom_z));
            e_offset += extrusion - original_e;
            line.set(E, relative_e ? extrusion : line.e() + e_offset, 5);
        } else if (!started && continuous && line.cmd_is("G1") && line.has_z() &&
                   std::abs(line.z() - top_z) < 0.001f) {
            // A single aligned vase needs no up/down motion at the layer seam.
            line.set(Z, bottom_z);
        } else if (inside && !relative_e && line.has_e()) {
            if (line.cmd_is("G92"))
                e_offset = 0.f;
            else
                line.set(E, line.e() + e_offset, 5);
        }
        if (inside && started && transition_out && !line.cmd_is("G1") &&
            !line.cmd_is("G0") && !line.cmd_is("G92"))
            finishing += line.raw() + '\n';
        output += line.raw() + '\n';
    });

    delete m_previous_layer;
    m_previous_layer = new std::vector<SpiralPoint>(std::move(current_layer));
    return output;
}

}
