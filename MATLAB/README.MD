# Omni Shared Memory Interface(for Only Linux)

## ▶️ Pre-Run

### C++ Side

```bash
./omni shm=777 hz=100
```

## Setup

Copy the following files into your working directory:
- `omni_shm_interface.cpp`
- `omni_shm_interface.hpp`

## Build (MATLAB Command)
Compile the interface using the following MATLAB command:
```matlab
mex omni_shm_interface.cpp
```

## Read (MATLAB Command)
To read data from shared memory, use the following MATLAB command:
```matlab
position = omni_shm_interface(777, 'read position');
disp(position);
```

```matlab
transform = omni_shm_interface(777, 'read transform');
disp(transform);
```

```matlab
joint = omni_shm_interface(777, 'read joint');
disp(joint);
```

```matlab
button = omni_shm_interface(777, 'read button');
disp(button);
```

## Write (MATLAB Command)
To write data to shared memory, use the following MATLAB command:
```matlab
omni_shm_interface(777, 'write', [0.5 0.5 0.5]);
```