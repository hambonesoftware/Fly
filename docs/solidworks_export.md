# SolidWorks Export Workflow

The Flythrough planner exposes an endpoint for exporting planned paths as
CSV files that can be imported into CAD tools such as SolidWorks.  By
default the endpoint exports the fully sampled path (the same points
used during animation and validation).  For users who wish to edit or
refine the path within SolidWorks, a "control points" mode is also
available.  This mode exports only the simplified or spline control
points from the slice‑based perimeter pipeline.

## Export Modes

The endpoint `GET /api/models/{model_id}/paths/{path_id}/export/solidworks` accepts an
optional query parameter `mode`.  Accepted values are:

| Mode             | Description                                                         |
|------------------|---------------------------------------------------------------------|
| `dense` (default)| Export all sampled points along the path.  Suitable for immediate use in downstream tools. |
| `controlPoints`  | Export only the simplified/spline control points.  Produces fewer points and makes it easier to adjust the path in SolidWorks.  Falls back to dense points if no control points are available (e.g. for orbit paths). |

Example URLs:

* Dense (default):

  ```
  /api/models/abc123/paths/def456/export/solidworks
  ```

* Control points:

  ```
  /api/models/abc123/paths/def456/export/solidworks?mode=controlPoints
  ```

## Importing the CSV into SolidWorks

1. In SolidWorks, open a new part and start a **3D sketch**.
2. From the **Tools** menu, choose **Spline Tools** > **Curve Through XYZ Points**.
3. Browse to the exported CSV and select it.  Ensure the delimiter is set to
   comma and that the header row is ignored.
4. Complete the wizard to import the points.  SolidWorks will create a
   spline passing through the imported points.  If you exported control
   points, the resulting spline will have the same control points used by
   the planner and can be adjusted easily in the graphics area.
5. After editing, you can measure or refine the curve as needed.

For more information on path planning and export options, see the
`docs/` directory and the project README.