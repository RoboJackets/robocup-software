# cbor2

Encode and parse data in the Concise Binary Object Representation (CBOR) data
format ([RFC8949](https://www.rfc-editor.org/rfc/rfc8949.html)).

This package supersedes [node-cbor](https://github.com/hildjj/node-cbor/tree/main/packages/cbor), with the following goals:

- Web-first.  Usable in Node and Deno.
- Simpler where possible.  Remove non-core features in favor of extensibility.
- Synchronous decoding.  Removes streaming capabilities that are rarely used.
- Complete break of API compatibility, allowing use of newer JavaScript constructs.
- No work-arounds for older environments.

## Start playing with cbor2 now

Visit the web [playground](https://hildjj.github.io/cbor2/playground/) for an
interactive view of how the library works.

## Supported Node.js versions

This project now only supports versions of Node.js that the Node team is
[currently supporting](https://github.com/nodejs/Release#release-schedule).
Currently, that means Node `20`+ is required.

## Installation

```bash
npm install --save cbor2
```

## Documentation

See the full API [documentation](http://hildjj.github.io/cbor2/).

Example:

```js
import {decode, diagnose, encode} from 'cbor2';

const encoded = encode(true); // Returns Uint8Array(1) [ 245 ]
decode(encoded); // Returns true

// Use integers as keys:
const m = new Map();
m.set(1, 25);
encode(m); // Returns Uint8Array(3) [ 161, 1, 24, 25 ]
diagnose(encode(m)); // {1: 25_0}
```

## Supported types

If you load `cbor2` using `import {decode, encode} from 'cbor2';`, all of the
type converters will be loaded automatically.  If you import only pieces of
the API and you want the type converters loaded, please also do
`import 'cbor2/types';`.

The following types are supported for encoding:

- boolean
- number (including -0, NaN, and ±Infinity)
- string (Set the wtf8 option to enable encoding invalid UTF-16 strings to tag 273)
- bigint
- Array
- Set
- Object (including null)
- Map
- undefined
- Date,
- RegExp
- URL
- TypedArrays: Uint8Array is mapped to a CBOR byte array, others are tagged
- Boxed primitive types: String, Number, BigInt, Boolean
- cbor2.Simple
- cbor2.Tag

Decoding supports the above types, including the following CBOR tag numbers:

| Tag | Generated Type      |
|-----|---------------------|
| 0   | Date                |
| 1   | Date                |
| 2   | BigInt              |
| 3   | BigInt              |
| 24  | any                 |
| 32  | URL                 |
| 33  | Tagged              |
| 34  | Tagged              |
| 35  | RegExp (deprecated) |
| 64  | Uint8Array          |
| 65  | Uint16Array         |
| 66  | Uint32Array         |
| 67  | BigUint64Array      |
| 68  | Uint8ClampedArray   |
| 69  | Uint16Array         |
| 70  | Uint32Array         |
| 71  | BigUint64Array      |
| 72  | Int8Array           |
| 73  | Int16Array          |
| 74  | Int32Array          |
| 75  | BigInt64Array       |
| 77  | Int16Array          |
| 78  | Int32Array          |
| 79  | BigInt64Array       |
| 81  | Float32Array        |
| 82  | Float64Array        |
| 85  | Float32Array        |
| 86  | Float64Array        |
| 258 | Set                 |
| 262 | any                 |
| 273 | string              |
| 21065 | RegExp            |
| 21066 | RegExp            |
| 55799 | any               |
| 0xffff | Error            |
| 0xffffffff | Error        |
| 0xffffffffffffffff | Error|

## Adding new Encoders

There are several ways to add a new encoder:

### `toCBOR` method

This is the easiest approach, if you can modify the class being encoded.  Add
a `toCBOR()` method to your class, which should return a two-element array
containing the tag number and data item that encodes your class.  If the tag
number is `NaN`, no tag will be written.  If you return undefined, nothing
will be written.  In this case you will likely write custom bytes to the Writer
instance that is passed in, perhaps using the encoding options.

For example:

```js
class Foo {
  constructor(one, two) {
    this.one = one;
    this.two = two;
  }

  toCBOR(_writer, _options) {
    return [64000, [this.one, this.two]];
  }
}
```

You can also modify an existing type by monkey-patching a `toCBOR` function
onto its prototype, but this isn't recommended.

### `toJSON()` method

If your object does not have a `toCBOR()` method, but does have a `toJSON()`
method, the value returned from `toJSON()` will be used to serialize the
object.

### `registerEncoder`

Sometimes, you want to support an existing type without modification to that
type.  In this case, call `registerEncoder(type, encodeFunction)`. The
`encodeFunction` takes an object instance and returns the same type as
`toCBOR` above:

```js
import {Buffer} from 'node:buffer';
import {registerEncoder} from 'cbor2/encoder';

registerEncoder(Buffer, b => [
  // Don't write a tag
  NaN,
  // New view on the ArrayBuffer, without copying bytes
  new Uint8Array(b.buffer, b.byteOffset, b.byteLength),
]);
```

If you want to have different encoders for different calls to `encode`, you
can pass a `types` parameter to `encode`:

```js
import {TypeEncoderMap, encode} from 'cbor2';
import {Buffer} from 'node:buffer';

const types = new TypeEncoderMap();
types.registerEncoder(Buffer, b => [
  // Don't write a tag
  NaN,
  // New view on the ArrayBuffer, without copying bytes
  new Uint8Array(b.buffer, b.byteOffset, b.byteLength),
]);
encode(Buffer.from('foo'), {types});
```

## Adding new decoders

Most of the time, you will want to add support for decoding a new tag type.  If
the Decoder class encounters a tag it doesn't support, it will generate a `Tag`
instance that you can handle or ignore as needed.  To have a specific type
generated instead, call `Tag.registerDecoder()` with the tag number and a function that will convert the tags value to the appropriate type. For the `Foo` example above, this might look like:

```js
import {Tag} from 'cbor2/tag';

Tag.registerDecoder(64000, tag => new Foo(tag.contents[0], tag.contents[1]));
```

You can also replace the default decoders by passing in an appropriate tag
function.  For example:

```js
// Tag 0 is Date/Time as an ISO-8601 string
import 'cbor2/types';
import {Tag} from 'cbor2/tag';

Tag.registerDecoder(0, ({contents}) => Temporal.Instant.from(contents));
```

Finally, you can pass a `tags` parameter to `decode` to specify tag decoding
for a single call to `decode`:

```js
const tags = new Map([
  [64000, ({contents}) => new Foo(contents[0], contents[1])],
]);
decode('d9fa00820102', {tags});
```

## Boxed Types

JavaScript "boxed" types, such as those created with `new Number(3)`, are
object wrappers around JavaScript primitives.  These objects can be used in
most places that a primitive value would be used, but since they are objects
they can have associated properties.  This library can decode CBOR values into
boxed types if desired, such that the original CBOR encoding of that value is
stored as a hidden property on the value, ensuring that round-tripping between
decoding and encoding will produce the original CBOR value, no matter which
of the many legal CBOR encodings were used for a given value.  For example:

```js
import {decode, encode} from 'cbor';

const val = decode('fa40800000', {boxed: true}); // [Number: 4]
encode(4); // 0x04
encode(new Number(4)); // 0x04
encode(val); // 0xfa40800000, the original encoding
encode(val, {ignoreOriginalEncoding: true}); // 0x04
```

When pedantic protocols are strict about the types they will accept, and you
want to force a particular numeric encoding, you can use the `encodedNumber`
function to create a boxed number with the desired encoding attached:

```js
import {encode, encodedNumber} from 'cbor';

const val = encodedNumber(4, 'i64'); // [Number: 4]
encode(val); // 0x1b0000000000000004
encode(encodedNumber(4)); // 0xf94400
```

Note that the default format for `encodedNumber` is the preferred floating
point representation, which can be explicitly selected with an encoding of `'f'`.

## CBOR Sequences

If you would like to decode a
[CBOR Sequence](https://www.rfc-editor.org/rfc/rfc8742.html),
use the `decodeSequence` method, which returns an iterator:

```js
import {decodeSequence} from 'cbor';

for (const item of decodeSequence('0102')) {
  console.log(item); // First 1, then 2
}
// Or
const seq = [...decodeSequence('0102')]; // [1, 2]
```

## Developers

The tests for this package use a set of test vectors from RFC 8949 appendix A
by importing a machine readable version of them from
[https://github.com/cbor/test-vectors](https://github.com/cbor/test-vectors).
For these tests to work, you will need to use the command
`git submodule update --init` after cloning or pulling this code.   See the
[git docs](https://gist.github.com/gitaarik/8735255#file-git_submodules-md)
for more information.

---
[![Build Status](https://github.com/hildjj/cbor2/workflows/Tests/badge.svg)](https://github.com/hildjj/cbor2/actions?query=workflow%3ATests)
[![codecov](https://codecov.io/gh/hildjj/cbor2/branch/main/graph/badge.svg?token=N7B7YLIDM4)](https://codecov.io/gh/hildjj/cbor2)
