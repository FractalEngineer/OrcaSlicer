# Spiral Vase islands

`spiral_mode_allow_islands` retains ordinary slice contours, including holes, while
allowing one complete outer perimeter to form a helix. Other perimeters and open
wall fragments print conventionally at the completed layer height. The option
defaults to false; the established single-contour Spiral Vase processor remains a
separate path.

## Primary contour identity

`GCode::process_layer()` selects the primary from the perimeter entities grouped
for emission. Both Classic and Arachne represent complete contours as
`ExtrusionLoop` objects, even when variable width or overhang treatment divides a
loop into multiple paths. Candidates must be outer loops with continuous,
extruding perimeter paths. Hole loops and open fragments are excluded.

Selection prefers the greatest overlap with the previous primary polygon, then
the largest enclosed area. Without overlap, the largest eligible loop wins. This
keeps the primary on the same body through ordinary area changes and allows
selection to continue across splits and merges. Layers without an eligible
contour clear the selection history.

When contours overlap, seam splitting uses the previous primary endpoint instead
of the nozzle's last island position, keeping island travel from rotating the
start of the helix around the body.

The selected entity is emitted before the ordinary island tour and skipped in
subsequent perimeter passes. Only that loop bypasses seam-gap clipping. It is
wrapped in internal `;_SPIRAL_VASE_BEGIN <bottom_z> <top_z>` and
`;_SPIRAL_VASE_END` comments. The scope includes its approach and extrusion but
ends before departure/wiping. Z bounds include the configured Z offset and use
the actual layer height, independently of travel lifts.

Selection state belongs to the serial G-code generator. The markers carry the
scope through the pipeline to the serial Spiral Vase filter; the filter does not
read the generator's changing loop pointer.

## Processing and motion

`SpiralVase` consumes the scope and measures only its extrusion length. Comments,
speed changes and zero-distance XY commands do not change contour identity. Z
interpolation, flow transitions and Smooth Spiral history apply only to the
selected loop. Tiny segments remain present.

Approach travel, retraction and lift commands remain intact. After a real travel,
the nozzle reaches the primary start before descending to the helix's bottom Z.
An aligned continuation with no XY travel holds the initial layer Z move at the
previous height. After the helix, ordinary island motion retains the writer's
completed-layer coordinates and lift behavior.

On the final layer, the finishing pass is emitted at the explicit end boundary, before any departure
motion. It contains only the primary loop. Absolute extrusion changes accumulate
an E-coordinate offset; an explicit `G92` restores the writer's expected E
coordinate before secondary paths resume. Replayed feedrates follow the motion
coordinates so cooling can remove redundant feedrates without dropping the
`G1` command.

A missing, incomplete, disconnected or degenerate scope leaves the layer's
ordinary motion intact and clears smoothing history. Such a layer never enters
the legacy processor, whose single-loop travel suppression is incompatible with
multiple contours. Internal scope comments are removed from exported G-code.

Contour splits, merges and open thin-wall fragments retain conventional island
behavior. Their local spacing can differ from the nominal layer height where
walls switch between the helix and flat paths; no extra finishing laps are
inserted to compensate for those geometry changes.
