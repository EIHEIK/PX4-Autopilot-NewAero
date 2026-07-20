# Honghu A1 V7 aerodynamic tables

Source: `鸿鹄翼A1样机仿真参数_V2.5(2).docx`, Word V2.5.

- `CL/CD/CY/Cm/Cl/Cn.csv`: main static 2D coefficient tables, rows are alpha deg, columns are beta deg. Only complete Word 2D rows alpha=-2..16 deg are used; incomplete alpha=18/20 beta=0-only rows are intentionally not fabricated.
- `control_tables/canard_*.csv`: canard CL/CD/Cm increment tables versus alpha and canard deflection.
- `control_tables/elevator_*.csv`: elevator CL/CD/Cm increment tables versus alpha and elevator deflection.
- `control_tables/aileron_*.csv`: aileron Cl/Cn/CD/CY increment tables derived from Word delta_a=+10 right-roll data and mirrored to -10/0/+10 deg.
- `control_tables/rudder_*.csv`: rudder Cl/Cn/CD/CY increment tables for delta_r=+10 deg versus alpha and signed beta; plugin scales linearly by actual rudder deflection.

The propulsion model is intentionally unchanged.
