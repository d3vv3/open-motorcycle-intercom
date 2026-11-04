# Third-Party Notices

## Espressif ESP Codec Device 2.0.0-beta5

Espressif Modified MIT License

Copyright (c) 2023-2026 Espressif Systems (Shanghai) CO., LTD

Permission is hereby granted for use EXCLUSIVELY with Espressif Systems products.
This includes the right to use, copy, modify, merge, publish, distribute, and sublicense
the Software, subject to the following conditions:

1. This Software MUST BE USED IN CONJUNCTION WITH ESPRESSIF SYSTEMS PRODUCTS.
2. The above copyright notice and this permission notice shall be included in all copies
   or substantial portions of the Software.
3. Redistribution of the Software in source or binary form FOR USE WITH NON-ESPRESSIF PRODUCTS
   is strictly prohibited.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT

## Espressif ESP Audio Codec 2.6.2

Provides the LC3 codec on the ESP32-S31.

Espressif Modified MIT License

Copyright (c) 2025 Espressif Systems (Shanghai) CO., LTD

Permission is hereby granted for use EXCLUSIVELY with Espressif Systems products.
This includes the right to use, copy, modify, merge, publish, distribute, and sublicense
the Software, subject to the following conditions:

1. This Software MUST BE USED IN CONJUNCTION WITH ESPRESSIF SYSTEMS PRODUCTS.
2. The above copyright notice and this permission notice shall be included in all copies
   or substantial portions of the Software.
3. Redistribution of the Software in source or binary form FOR USE WITH NON-ESPRESSIF PRODUCTS
   is strictly prohibited.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT

## SpeexDSP Resampler 1.2.1

Vendored from `xiph/speexdsp` commit
`1b28a0f61bc31162979e1f26f3981fc3637095c8` (release `SpeexDSP-1.2.1`).
The source archive SHA-256 is
`59a53188f6d6ca2b48752f533523986a0604d2526eb50789b8fb09aa6b12645f`.

Copyright 2002-2008 Xiph.org Foundation
Copyright 2002-2008 Jean-Marc Valin
Copyright 2005-2007 Analog Devices Inc.
Copyright 2005-2008 Commonwealth Scientific and Industrial Research Organisation (CSIRO)
Copyright 1993, 2002, 2006 David Rowe
Copyright 2003 EpicGames
Copyright 1992-1994 Jutta Degener, Carsten Bormann

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

- Neither the name of the Xiph.org Foundation nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE FOUNDATION OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## embassy-nrf-esb 0.1.0 (vendored, nRF52840 firmware)

Vendored in `rust/vendor/embassy-nrf-esb`, under MIT OR Apache-2.0
(`LICENSE-MIT`, `LICENSE-APACHE-2.0`). Portions are adapted from
[esb-ng](https://github.com/jamesmunns/esb), Copyright (c) James Munns and
esb-ng contributors, licensed under MIT OR Apache-2.0.
See [its NOTICE](rust/vendor/embassy-nrf-esb/NOTICE.md) for details.

The nRF firmware does not enable the crate's `mpsl` feature, so no Nordic
MPSL or SoftDevice Controller binaries are linked.

## nrf-pac 0.3.0 (nRF52840 firmware)

Register definitions generated from Nordic Semiconductor's SVD files.

Copyright (c) 2010 - 2024, Nordic Semiconductor ASA All rights reserved.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

## Other Rust crates (nRF52840 firmware)

The remaining crates.io dependencies, pinned in
`rust/nrf-embassy-firmware/Cargo.lock`, are permissively licensed, mainly
MIT OR Apache-2.0. Each crate's license travels with its source.
