/* ---------------------------------------------------------------------------
** videoHDR_Record.h
** This file implements the records map to be used by the VideoHDR integrator
**
** Author: Ricardo Marques (based on photonmap.cpp)
** Date: March, 2014
** -------------------------------------------------------------------------*/

#include <mitsuba/render/videoHDR_RecordMap.h>
#include <mitsuba/render/scene.h>
//#include <mitsuba/render/phase.h>
#include <fstream>

MTS_NAMESPACE_BEGIN

videoHDR_RecordMap::videoHDR_RecordMap(size_t recordsCount) : 
	m_kdtree(0, VideoHDR_RecordTree::ESlidingMidpoint) 
{
	m_kdtree.reserve(recordsCount);
}

videoHDR_RecordMap::videoHDR_RecordMap(Stream *stream, InstanceManager *manager)
    : SerializableObject(stream, manager),
	  m_kdtree(0, VideoHDR_RecordTree::ESlidingMidpoint) 
{
	m_kdtree.resize(stream->readSize());
	m_kdtree.setDepth(stream->readSize());
	m_kdtree.setAABB(AABB(stream));
	for (size_t i=0; i<m_kdtree.size(); ++i)
		m_kdtree[i] = VideoHDR_Record(stream);
}

void videoHDR_RecordMap::serialize(Stream *stream, InstanceManager *manager) const 
{
	Log(EDebug, "Serializing a HDR record map (%s)",
		memString(m_kdtree.size() * sizeof(VideoHDR_Record)).c_str());
	stream->writeSize(m_kdtree.size());
	stream->writeSize(m_kdtree.getDepth());
	m_kdtree.getAABB().serialize(stream);
	for (size_t i=0; i<m_kdtree.size(); ++i)
		m_kdtree[i].serialize(stream);
}

videoHDR_RecordMap::~videoHDR_RecordMap() 
{}

std::string videoHDR_RecordMap::toString() const 
{
	std::ostringstream oss;
	oss << "VideoHDR_RecordMap[" << endl
		<< "  size = " << m_kdtree.size() << "," << endl
		<< "  capacity = " << m_kdtree.capacity() << "," << endl
		<< "  aabb = " << m_kdtree.getAABB().toString() << "," << endl
		<< "  depth = " << m_kdtree.getDepth() << endl
		<< "]";
	return oss.str();
}
void videoHDR_RecordMap::dumpOBJ(const std::string &filename) 
{
	std::ofstream os(filename.c_str());
	os << "o VideoHDR Records" << endl;
	for (size_t i=0; i<m_kdtree.size(); ++i) {
		const Point &p = m_kdtree[i].getPosition();
		os << "v " << p.x << " " << p.y << " " << p.z << endl;
	}

	/// Need to generate some fake geometry so that blender will import the points
	for (size_t i=3; i<=m_kdtree.size(); i++)
		os << "f " << i << " " << i-1 << " " << i-2 << endl;
	os.close();
}


MTS_IMPLEMENT_CLASS_S(videoHDR_RecordMap, false, SerializableObject)
MTS_NAMESPACE_END
