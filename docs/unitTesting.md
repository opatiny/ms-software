# PlatformIO unit testing system

- tests hierarchy: https://docs.platformio.org/en/latest/advanced/unit-testing/structure/hierarchy.html
- Unity test framework: https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/unity.html

## Debugging unit tests

https://community.platformio.org/t/how-to-debug-a-unit-test-native/20899/7

## Running tests locally

You apparently can run tests locally on the computer instead of on the board using the `native` environment.

https://docs.platformio.org/en/stable/platforms/native.html#platform-native

## Double precision tests

https://community.platformio.org/t/teensy-4-0-unity-enable-double-precision-fp-asserts/17240

If you get the error "unity double precision disabled" at this line to `platformio.ini`:

```
build_flags = -D UNITY_INCLUDE_DOUBLE -DUNITY_DOUBLE_PRECISION=1e-12
```
