# Architecture decision records

Accepted decisions for Intrinsic Core live in this directory. Each record
cites the source it was taken from. The PDR text checked in beside a record
is evidence, not an implementation.

| ADR | Status | Title |
| --- | --- | --- |
| [0001](0001-multi-embodiment-capability-architecture.md) | Accepted | Additive multi-embodiment capability architecture |

Source for ADR 0001:

- [Intrinsic-Core-Multi-Embodiment-PDR.txt](assets/Intrinsic-Core-Multi-Embodiment-PDR.txt)
- [Intrinsic-Core-Multi-Embodiment-PDR.docx](assets/Intrinsic-Core-Multi-Embodiment-PDR.docx)

ADR 0001 records the decision and does not add packages. Issue #15 adds the
stamped header, validity companion, and ENU/NED frame policy. Issue #16 adds
the embodiment capability descriptor in that same package. Issue #17 adds
vehicle state and command protos under `intrinsic/vehicle/proto`. Empty
messages stay opt-in and leave the manipulator baseline unchanged. Issue #21
adds the opt-in `intrinsic_vehicle` package scaffold. Real-time and
soft-real-time ownership for those packages is documented with them.

- [Embodiment proto README](../../intrinsic_apis/intrinsic/embodiment/proto/README.md)
- [Vehicle proto README](../../intrinsic_apis/intrinsic/vehicle/proto/README.md)
- [Vehicle package README](../../intrinsic_vehicle/intrinsic/vehicle/README.md)
