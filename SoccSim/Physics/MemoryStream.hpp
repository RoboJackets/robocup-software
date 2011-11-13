#pragma once

#include <NxPhysics.h>
#include <NxStream.h>

#include <vector>
#include <stdint.h>

class MemoryStream : public NxStream
{
public:
	MemoryStream();

	virtual NxU8 readByte() const; // FIXME: fails here
	virtual NxU16 readWord() const;
	virtual NxU32 readDword() const;
	virtual NxF32 readFloat() const; // FIXME: also possible fail here
	virtual NxF64 readDouble() const;
	virtual void readBuffer(void* buffer, NxU32 size) const;

	virtual NxStream& storeByte(NxU8 b);
	virtual NxStream& storeWord(NxU16 w);
	virtual NxStream& storeDword(NxU32 d);
	virtual NxStream& storeFloat(NxReal f); // FIXME: possible fail here
	virtual NxStream& storeDouble(NxF64 f);
	virtual NxStream& storeBuffer(const void* buffer, NxU32 size);

private:
	template <typename T>
	T read() const
	{
		T res = 0;
		assert((_readLoc + sizeof(T)) < _data.size());
		memcpy(&res, &_data[_readLoc], sizeof(T));
		_readLoc += sizeof(T);

		return res;
	}

private:
	mutable std::vector<uint8_t> _data;
	mutable NxU32 _readLoc;
};
