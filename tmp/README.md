# Running

```sh
cd libfranka

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DSTRICT=ON -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON

make -j X run_all_tests generate_joint_position_motion

# Run test to make sure dumb stuff isn't being dumb.
./test/run_all_tests

# Record data.
( set -eux;
host=maverick-panda
for delay_style in time buffer; do
    ./examples/generate_joint_position_motion ${host} ${delay_style} 0
    ./examples/generate_joint_position_motion ${host} ${delay_style} 1
    ./examples/generate_joint_position_motion ${host} ${delay_style} 50
done
)
