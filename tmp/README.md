# Testing out jerky position motion w/ delays?

*Goal*: Do first- or whatever-order-hold. However, just delaying (ZOH) makes
jerky stuff.

## Building

```sh
cd libfranka

mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DSTRICT=ON \
    -DBUILD_COVERAGE=OFF \
    -DBUILD_DOCUMENTATION=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TESTS=ON

# Build specific targets.
make -j run_all_tests generate_joint_position_motion

# Run test to make sure my (Eric's) stuff isn't being dumb.
./test/run_all_tests
```

## Running

Just copy+paste before parentheses.

```sh
cd libfranka/build

( set -eux;
host=maverick-panda
for delay_style in time buffer; do
    ./examples/generate_joint_position_motion ${host} ${delay_style} 0
    ./examples/generate_joint_position_motion ${host} ${delay_style} 1
    ./examples/generate_joint_position_motion ${host} ${delay_style} 50
done
)
```

## Plotting

This consumes the plots that were produced above. You may need to run
`sudo apt install python3-venv` first.

```sh
cd libfranka/tmp
./run.sh jupyter lab ./plot_joints_libfranka.ipynb
```
