/* ---------------------------------------------------------------------------
** videoHDR_Record.h
** This file implements the records to be used to fill the VideoHDR_RecordMap
**
** Author: Ricardo Marques (based on photon.cpp)
** Date: March, 2014
** -------------------------------------------------------------------------*/

#include <mitsuba/render/videoHDR_Record.h>

MTS_NAMESPACE_BEGIN

VideoHDR_Record::VideoHDR_Record(Stream *stream) {
	position = Point(stream);
	setRightIndex(0, stream->readUInt());
	data.irradiance = Spectrum(stream);
	data.shapeIndex = stream->readUInt();
	data.n = Normal(stream);
	flags = stream->readUChar();
}

void VideoHDR_Record::serialize(Stream *stream) const {
	position.serialize(stream);
	stream->writeUInt(getRightIndex(0));
	data.irradiance.serialize(stream);
	stream->writeUInt(data.shapeIndex);
	data.n.serialize(stream);
	stream->writeUChar(flags);
}

VideoHDR_Record::VideoHDR_Record(const Point &pos, const Spectrum &irradiance_, const size_t &shapeIndex_, const Normal &n_) {
	position = pos;
	data.irradiance = irradiance_;
	data.shapeIndex = shapeIndex_;
	data.n = n_;
	flags = 0;
}

std::string VideoHDR_Record::toString() const {
	std::ostringstream oss;
	oss << "VideoHDR Record[" << endl
		<< "  pos = " << getPosition().toString() << "," << endl
		<< "  irradiance = " << getIrradiance().toString() << "," << endl
		<< "  shapeIndex = " << getShapeIndex() << "," << endl
		<< "  normal " << getNormal().toString() << endl
		<< "]";
	return oss.str();
}


MTS_NAMESPACE_END
