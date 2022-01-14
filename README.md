# RalOS

A simple OS built from scratch, which is the final project of ZJU *Operating System* course.  

## Feature List

- Bootstrap
- Interrupt handling
- Process scheduling
- Virtual memory
- User mode
- Page fault handling and `fork` (hasn't been implemented yet)

## Highlights

- Many core procedures are implemented using C++ as the programming language
- You can find reasonable usages of some modern C++ features, like enumeration base, `decltype`, `consteval`, `[[fallthrough]]` and so on
- Compile with `-Werror -Wall -Wextra` tags

## Build

Firstly, pull `alphavake/oslab` and start a container from this image. This image includes build tools, as well as Qemu.  

```shell
docker pull alphavake/oslab
# then start a container from this image
```

Then, clone this repository in the container, and build it.  

```shell
git clone https://github.com/RalXYZ/RalOS.git
cd RalOS
make        # build
make run    # build and run
```

![lab5](https://raw.githubusercontent.com/RalXYZ/repo-pictures/main/RalOS/lab5.png)

## Credit

This project is based on the work of [zjusec/os21fall](https://gitee.com/zjusec/os21fall) on gitee.  
