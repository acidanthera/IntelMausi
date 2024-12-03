IntelMausi Changelog
====================
#### v1.0.8
- Minor fixes found by static analysis

#### v1.0.7
- Added force WOL support (`mausi-force-wol` device property or `-mausiwol` boot argument)

#### v1.0.6
- Fixed loading on 10.11 and earlier (regressed in 1.0.5)

#### v1.0.5
- Merged changes from 2.5.3d1
- Updated e1000e sources from Linux upstream branch
- Solved high-load throttling problem for I219 family
- Add support for new I219 Cannon-Point family devices
    * I219-LM13
    * I219-V13
    * I219-LM14
    * I219-V14
    * I219-LM15
    * I219-V15
    * I219-LM16
    * I219-V16
    * I219-LM17
    * I219-V17
    * I219-LM18
    * I219-V18
    * I219-LM19
    * I219-V19
- Fixed WoL on I219 family devices
- Use Random RSS key hash generation instead of static
- Fix IRQ mask for Cannon-Point devices
- Adjusted debug log

#### v1.0.4
- Added MacKernelSDK with Xcode 12 compatibility
- Added IntelSnowMausi variant for 10.6-10.8

#### v1.0.3
- Merged changes from 2.5.1d1

#### v1.0.2
- Merged changes from 2.5.0d14

#### v1.0.1
- Unified release archive names
- Fixed loading on 10.9

#### v1.0.0
- Initial release
