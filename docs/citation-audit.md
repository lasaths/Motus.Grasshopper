# Citation audit (P8)

Motus.NET method provenance snapshot. Canonical: upstream [`docs/METHODS.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/METHODS.md) · [`REFERENCES.bib`](https://github.com/lasaths/Motus.NET/blob/master/docs/REFERENCES.bib) · local [motus-net/METHODS.md](motus-net/METHODS.md). Pin: NuGet **0.13.2**.

| Area | Primary | DOI | SOTA alts (documented, not required) |
|------|---------|-----|--------------------------------------|
| TOTG | TOPP-RA (Pham & Pham 2018) | 10.1109/TRO.2018.2819195 | TOPP NI (Kunz & Stilman); CO-TOPP |
| RRT-Connect | Kuffner & LaValle 2000 | (ICRA) | RRT* (Karaman & Frazzoli) when native |
| PRM* | Karaman & Frazzoli 2011 | 10.1177/0278364911406761 | BIT*/AIT* native IDs |
| CHOMP-lite | Zucker et al. 2013 | 10.1177/0278364913488805 | TrajOpt 10.1177/0278364914528132; GPMP2 10.1177/0278364918790369 |
| Constraints | MoveIt-shaped / OMPL seam | 10.1109/MRA.2012.2205651 | Task-space regions |
| PoE FK/IK | Lynch & Park | 10.1017/9781316095072 | — |
| Stewart | Merlet; Dasgupta & Mruthyunjaya | 10.1007/1-4020-4133-0; 10.1016/S0094-114X(99)00006-3 | Closed-form FK variants |
| Legged | LeggedMethodRefs stack | Song&Waldron; McGhee&Frank; Lynch&Park | Bretl & Lall 10.1109/TRO.2008.2001360 (out of scope) |
| SE2 mobility | Holonomic SE2 + RRT-Connect | LaValle *Planning Algorithms* | Nonholonomic / Reeds–Shepp (out of scope) |
