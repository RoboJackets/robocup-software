# @cto.af/wtf8

Encode and decode [WTF-8](https://simonsapin.github.io/wtf-8/) with a similar
API to
[TextEncoder](https://developer.mozilla.org/en-US/docs/Web/API/TextEncoder)
and
[TextDecoder](https://developer.mozilla.org/en-US/docs/Web/API/TextDecoder).

The goal is to be able to parse and generate bytestreams that can store any
JavaScript string, including ones that have unpaired surrogates.

## Installation

```sh
npm install @cto.af/wtf8
```

## API

Full [API documentation](http://cto-af.github.io/wtf8/) is available.

Example:

```js
import {Wtf8Decoder, Wtf8Encoder} from '@cto.af/wtf8';

const bytes = new Wtf8Encoder().encode('\ud800');
const string = new Wtf8Decoder().decode(bytes); // '\ud800'
```

W3C streams are also provided: `Wtf8EncoderStream` and `Wtf8DecoderStream`.

## Notes

Used a few of the tricks from the paper
[Validating UTF-8 In Less Than One Instruction Per Byte](https://arxiv.org/pdf/2010.03090),
but not all of them.  Moving data in and out of WASM to be able to use SIMD
might be slightly faster, but since we're not merely validating but instead
actually decoding (and generating replacement characters when fatal is false),
staying in JS seems good enough for the moment.

---
[![Build Status](https://github.com/cto-af/wtf8/workflows/Tests/badge.svg)](https://github.com/cto-af/wtf8/actions?query=workflow%3ATests)
[![codecov](https://codecov.io/gh/cto-af/wtf8/branch/main/graph/badge.svg?token=N7B7YLIDM4)](https://codecov.io/gh/cto-af/wtf8)
