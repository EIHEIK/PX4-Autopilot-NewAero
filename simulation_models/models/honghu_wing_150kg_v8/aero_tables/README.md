# Honghu A1 V8 aerodynamic data

The CSV values are transcribed from the V2.5(2) PDF and interpreted in the
PDF/PX4 FRD convention. Static tables contain dimensionless coefficients.
Control tables contain local coefficient derivatives per degree; the V8 core
multiplies them by signed `delta_doc_deg`. No zero-valued derivative column is
inserted. Derived alpha=18/20 rows are identified in `data_provenance.yaml`.
