#include "MemoryStream.hpp"
#include <stdio.h>

MemoryStream::MemoryStream() : _readLoc(0)
{
}

// Loading API
NxU8 MemoryStream::readByte() const
{
	//    NxU8 b = read<NxU8>();
	//    return b;

//	printf("In MemoryStream::readByte(): start\n");
	NxU8 res = 0;
	const uint32_t s = sizeof(NxU8);
	assert(s == 1);
	assert((_readLoc + s) < _data.size());
	memcpy(&res, &_data[_readLoc], s);
	_readLoc += s;

//	printf("In MemoryStream::readByte(): end\n");
	return res;
}

NxU16 MemoryStream::readWord() const
{
	//    NxU16 w = read<NxU16>();
	//    return w;

	NxU16 res = 0;
	const uint32_t s = sizeof(NxU16);
	assert(2 == s);
	assert((_readLoc + s) < _data.size());
	memcpy(&res, &_data[_readLoc], s);
	_readLoc += s;
	return res;
}

NxU32 MemoryStream::readDword() const
{
//	NxU32 d = read<NxU32>();
//	return d;

	NxU32 res = 0;
	const uint32_t s = sizeof(NxU32);
	assert(s == 4);
	assert((_readLoc + s) < _data.size());
	memcpy(&res, &_data[_readLoc], s);
	_readLoc += s;
	return res;
}

NxF32 MemoryStream::readFloat() const
{
//	float f = read<float>();
//	return f;

	NxF32 res = 0;
	const uint32_t s = sizeof(NxF32);
	assert(s == 4);
	assert((_readLoc + s) < _data.size());
	memcpy(&res, &_data[_readLoc], s);
	_readLoc += s;
	return res;
}

NxF64 MemoryStream::readDouble() const
{
//	NxF64 f = read<NxF64>();
//	return f;

	NxF64 res = 0;
	const uint32_t s = sizeof(NxF64);
	assert(s == 8);
	assert((_readLoc + s) < _data.size());
	memcpy(&res, &_data[_readLoc], s);
	_readLoc += s;
	return res;
}

void MemoryStream::readBuffer(void* buffer, NxU32 size) const
{
	uint8_t* bytes = (uint8_t*)buffer;

	memcpy (bytes, &_data[_readLoc], size);
	_readLoc += size;
}

// Saving API
NxStream& MemoryStream::storeByte(NxU8 b)
{
	storeBuffer(&b, sizeof(b));
	return *this;
}

NxStream& MemoryStream::storeWord(NxU16 w)
{
	storeBuffer(&w, sizeof(w));
	return *this;
}

NxStream& MemoryStream::storeDword(NxU32 d)
{
	storeBuffer(&d, sizeof(d));
	return *this;
}

NxStream& MemoryStream::storeFloat(NxReal f)
{
	storeBuffer(&f, sizeof(f));
	return *this;
}

NxStream& MemoryStream::storeDouble(NxF64 f)
{
	storeBuffer(&f, sizeof(f));
	return *this;
}

NxStream& MemoryStream::storeBuffer(const void* buffer, NxU32 size)
{
	const size_t length = _data.size();
	_data.resize(length + size);
	memcpy(&_data[length], buffer, size);

	return *this;
}
