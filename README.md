# Arm PrimeCell GPIO (PL061) driver

Driver implementation for the [PL061 GPIO peripheral](https://developer.arm.com/documentation/ddi0190/latest/).

## Implemented features

- Configuring pins as inputs or outputs.
- Read and writing pins, individually or together.
- Configuring interrupts for pins, and reading interrupt status.
- Reading peripheral ID registers.

## Feature flags

- `embedded-hal`: Adds implementations of [`embedded-hal`] traits for `Pin`. This is enabled by
  default.

## License

The project is MIT and Apache-2.0 dual licensed, see `LICENSE-Apache-2.0` and `LICENSE-MIT`.

## Maintainers

arm-pl061 is a trustedfirmware.org maintained project. All contributions are ultimately merged by the maintainers
listed below.

- Bálint Dobszay <balint.dobszay@arm.com>
  [balint-dobszay-arm](https://github.com/balint-dobszay-arm)
- Imre Kis <imre.kis@arm.com>
  [imre-kis-arm](https://github.com/imre-kis-arm)
- Sandrine Afsa <sandrine.afsa@arm.com>
  [sandrine-bailleux-arm](https://github.com/sandrine-bailleux-arm)
- Andrew Walbran <qwandor@google.com> [qwandor](https://github.com/qwandor)
- Jeremi Miller <jeremimiller@google.com> [jrm224](https://github.com/jrm224)

## Contributing

Please follow the directions of the [Trusted Firmware Processes](https://trusted-firmware-docs.readthedocs.io/en/latest/generic_processes/index.html)

Contributions are handled through [review.trustedfirmware.org](https://review.trustedfirmware.org/q/project:arm-firmware-crates/arm-pl061).

## Arm trademark notice

Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates).

This project uses some of the Arm product, service or technology trademarks, as listed in the
[Trademark List][1], in accordance with the [Arm Trademark Use Guidelines][2].

Subsequent uses of these trademarks throughout this repository do not need to be prefixed with the
Arm word trademark.

[1]: https://www.arm.com/company/policies/trademarks/arm-trademark-list
[2]: https://www.arm.com/company/policies/trademarks/guidelines-trademarks
[`embedded-hal`]: https://crates.io/crates/embedded-hal

---

_Copyright The arm-pl061 Contributors._
