## Build
```
g++ -o data data.cpp -std=gnu++20 -O3
```

## Usage
```
Usage: ./data <base_capture_rate> <base_flee_rate> <safari_ball_rate> <initial_ball_cnt> [method]
  methods:
    naive [max_rounds=10]
    dp [max_rounds=10]
    lp
  default: lp
```

## Example
```
./data 190 90 1.5 30 dp 1000
```
