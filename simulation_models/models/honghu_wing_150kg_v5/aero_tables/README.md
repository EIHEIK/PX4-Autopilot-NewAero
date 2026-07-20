# Honghu 150kg V5 aero tables

These CSV files are generated from `鸿鹄翼A1样机仿真参数_V2.5(2)_extracted.txt`.

- Rows: angle of attack `alpha_deg` in degrees.
- Columns: sideslip `beta_deg` in degrees.
- The source tables contain non-negative beta values; the V5 plugin uses absolute beta for CL/CD/Cm and mirrors odd lateral-directional coefficients for negative beta.
- First-stage V5 uses these main static tables plus Word dynamic derivatives; control-surface effects remain the linear V3 terms.
