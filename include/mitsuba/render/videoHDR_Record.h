/* ---------------------------------------------------------------------------
** videoHDR_Record.h
** This file implements the records to be used to fill the VideoHDR_RecordMap
**
** Author: Ricardo Marques (based on photon.h)
** Date: March, 2014
** -------------------------------------------------------------------------*/

#pragma once
#if !defined(__MITSUBA_RENDER_VIDEOHDR_RECORD_H_)
#define __MITSUBA_RENDER_VIDEOHDR_RECORD_H_

#include <mitsuba/core/serialization.h>
#include <mitsuba/core/kdtree.h>

// Unbalanced record map using the sliding midpoint rule.
//#define MTS_PHOTONMAP_LEFT_BALANCED 0

MTS_NAMESPACE_BEGIN

/// Internal data record used by \ref VideoHDR_Record
struct VideoHDR_RecordData {
	Spectrum irradiance;
	size_t shapeIndex;
	Normal n;
};

/// Struct VideoHDR_Record
struct MTS_EXPORT_RENDER VideoHDR_Record :
	public SimpleKDNode<Point, VideoHDR_RecordData> {

	//friend class PhotonMap;
public:
	/// Dummy constructor
	inline VideoHDR_Record() { }

	/// Construct from a position and an irradiance
	VideoHDR_Record(const Point &pos, const Spectrum &irradiance, const size_t& shapeIndex, const Normal &n);

	/// Unserialize from a binary data stream
	VideoHDR_Record(Stream *stream);

	/// Return the normal of the record
	inline Normal getNormal() const {
		return data.n;
	}

	/// Return the irrandiance of the record
	inline Spectrum getIrradiance() const {
		return data.irradiance;
	}
	
	/// Return the irrandiance of the record
	inline void setIrradiance(Spectrum const &spec) {
		data.irradiance = spec;
	}

	/// Return the shape index of the record
	inline size_t getShapeIndex() const {
		return data.shapeIndex;
	}

	/// Serialize to a binary data stream
	void serialize(Stream *stream) const;

	/// Return a string representation (for debugging)
	std::string toString() const;
protected:

};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_VIDEOHDR_RECORD_H_ */
