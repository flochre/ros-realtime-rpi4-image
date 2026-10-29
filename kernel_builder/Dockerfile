FROM ubuntu:noble

USER root
ARG DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s -f /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update && apt-get install -q -y tzdata apt-utils lsb-release software-properties-common openssh-client \
    && rm -rf /var/lib/apt/lists/*

ARG ARCH=arm64
ARG UNAME_R
ARG RT_PATCH
ARG triple=aarch64-linux-gnu
ARG KERNEL_VERSION=6.8.0
ARG UBUNTU_VERSION=noble
ARG LTTNG_VERSION=2.13
ARG KERNEL_DIR=linux-raspi

# Setup cross-compile architecture
RUN apt-get update && apt-get install -q -y \
    gcc-${triple} \
    && dpkg --add-architecture ${ARCH} \
    && sed -i 's/deb h/deb [arch=amd64] h/g' /etc/apt/sources.list \
    && sed -i 's/deb h/deb [arch=amd64] h/g' /etc/apt/sources.list.d/* \
    && sed -i '/Components/a\Architectures: amd64' /etc/apt/sources.list.d/*.sources \
    && echo "deb [arch=$ARCH] http://ports.ubuntu.com/ubuntu-ports/ $(lsb_release -s -c) main universe restricted" >> /etc/apt/sources.list.d/ubuntu-ports.list \
    && echo "deb-src [arch=$ARCH] http://ports.ubuntu.com/ubuntu-ports/ $(lsb_release -s -c) main universe restricted" >> /etc/apt/sources.list.d/ubuntu-ports.list \
    && echo "deb [arch=$ARCH] http://ports.ubuntu.com/ubuntu-ports $(lsb_release -s -c)-updates main universe restricted" >> /etc/apt/sources.list.d/ubuntu-ports.list \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV UID=1000
ENV GID=1000

# Install kernel build dependencies
RUN apt-get update && apt-get build-dep -q -y linux \
    && apt-get install -q -y \
    libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf \
    fakeroot wget curl gzip git bash-completion time \
    && rm -rf /var/lib/apt/lists/*

# Configure user
RUN userdel -r ubuntu || true \
    && groupadd -g $GID user \
    && useradd -m -d /home/user -s /bin/bash -u $UID -g $GID user \
    && apt-get update && apt-get install -q -y sudo \
    && gpasswd -a user sudo \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo 'user\nuser\n' | passwd user \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /linux_build && chown -R user:user /linux_build

USER user
WORKDIR /linux_build

# Detect latest raspi kernel UNAME_R
RUN if test -z "$UNAME_R"; then UNAME_R=`curl -s http://ports.ubuntu.com/pool/main/l/linux-raspi/ | grep linux-buildinfo | grep -o -P '(?<=<a href=").*(?=">l)' | grep ${ARCH} | grep ${KERNEL_VERSION} | sort | tail -n 1 | cut -d '-' -f 3-4`-raspi; fi \
    && echo $UNAME_R > /home/user/uname_r

# Clone Ubuntu raspi kernel git repository
RUN git config --global https.postBuffer 1048576000 \
    && git clone -b master --depth 1 --single-branch --jobs=$(($(nproc) * 2)) --progress https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux-raspi/+git/${UBUNTU_VERSION} ${KERNEL_DIR} \
    && cd ${KERNEL_DIR} \
    && git fetch --tag

# Checkout matching tag
RUN cd /linux_build/${KERNEL_DIR} \
    && git tag -l *`cat /home/user/uname_r | cut -d '-' -f 2`* | sort -V | tail -1 > /linux_build/tag \
    && git checkout `cat /linux_build/tag`

# Retrieve base Ubuntu raspi kernel config from buildinfo
RUN cd /home/user \
    && wget http://ports.ubuntu.com/pool/main/l/linux-raspi/linux-buildinfo-${KERNEL_VERSION}-`cat /home/user/uname_r | cut -d '-' -f 2`-raspi_${KERNEL_VERSION}-`cat /linux_build/tag | cut -d '-' -f 4`_${ARCH}.deb \
    && dpkg -X *.deb /home/user/

COPY --chown=user:user getpatch.sh patch_kernel_rt.py /home/user/

# Retrieve and apply matching PREEMPT_RT patch and compatibility fixes
RUN cd /linux_build/${KERNEL_DIR} \
    && if test -z "$RT_PATCH"; then /home/user/getpatch.sh `make kernelversion` > /home/user/rt_patch; else echo "$RT_PATCH" > /home/user/rt_patch; fi \
    && cd /linux_build \
    && wget http://cdn.kernel.org/pub/linux/kernel/projects/rt/`echo ${KERNEL_VERSION} | cut -d '.' -f 1-2`/older/patch-`cat /home/user/rt_patch`.patch.gz \
    && gunzip patch-`cat /home/user/rt_patch`.patch.gz \
    && cd /linux_build/${KERNEL_DIR} \
    && OUT="$(patch -p1 --forward < ../patch-`cat /home/user/rt_patch`.patch)" || echo "${OUT}" | grep "Skipping patch" -q || (echo "$OUT" && false) \
    && /home/user/patch_kernel_rt.py .

# Copy custom config fragment with PREEMPT_RT and in-tree drivers (e.g. LSM6DSX)
COPY --chown=user:user .config-fragment /linux_build/.

# Merge kernel configuration
RUN cd /linux_build/${KERNEL_DIR} \
    && cp /home/user/usr/lib/linux/`cat /home/user/uname_r`/config .config \
    && ARCH=${ARCH} CROSS_COMPILE=${triple}- ./scripts/kconfig/merge_config.sh .config /linux_build/.config-fragment \
    && fakeroot debian/rules clean

# Apply custom Yocto-style kernel patches from kernel_builder/patches/*.patch
COPY --chown=user:user patches/ /linux_build/patches/
RUN cd /linux_build/${KERNEL_DIR} \
    && for p in $(find /linux_build/patches -maxdepth 1 -name "*.patch" | sort); do \
         echo "==> Applying patch: $(basename "$p")"; \
         patch -p1 --forward < "$p"; \
       done

CMD ["/bin/bash"]

