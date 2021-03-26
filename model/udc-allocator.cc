/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "udc-allocator.h"

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/vector.h"
#include "ns3/mobility-model.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <utility>

#include <CGAL/squared_distance_3.h>

namespace ns3 {


//
// This is the start of where I have been adding
// implementation code for the UDCPositionAllocator.
// The current algorithm is by Ghosh et al.
//

NS_OBJECT_ENSURE_REGISTERED (UDCPositionAllocator);

struct pair_hash {
    inline size_t operator()(const std::pair<int,int> &v) const {
        return v.first*31+v.second;
    }
};

inline bool
isEven(int n) {
    return (n % 2 == 0);
}

inline bool
isPresent(const std::unordered_set<std::pair<int,int>, pair_hash> &table, const int vertical, const int horizontal) {
    return table.find(std::make_pair(vertical,horizontal)) != table.end();
}

bool
operator!= (const Vector& l, const Vector& r) {
	return l.x == r.y && l.y == r.y && l.z == r.z;
}

UDCPositionAllocator::Point_3
CreatePointFromVector (const Vector& v) {
	return UDCPositionAllocator::Point_3(v.x,v.y,v.z);
}

double
UDCPositionAllocator::SquaredDistance (const Vector& l, const Vector& r) {
	return CGAL::squared_distance(
			CreatePointFromVector (l),
			CreatePointFromVector (r)
	);
}

TypeId
UDCPositionAllocator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UDCPositionAllocator")
    .SetParent<PositionAllocator> ()
    .SetGroupName ("Mobility")
    .AddConstructor<UDCPositionAllocator> ()
  ;
  return tid;
}

void
UDCPositionAllocator::SetSites (NodeContainer c)
{
	NodeContainer::Iterator i = c.Begin ();
	Vector firstPosition = (*i)->GetObject<MobilityModel> ()->GetPosition ();

	double MinX = firstPosition.x,
		   MaxX = firstPosition.x,
		   MinY = firstPosition.y,
		   MaxY = firstPosition.y,
		   MinZ = firstPosition.z,
		   MaxZ = firstPosition.z;


	m_sites.reserve (c.GetN());
	using std::min;
	using std::max;

	for ( ; i != c.End (); ++i)
	{
		Ptr<MobilityModel> edMobility = (*i)->GetObject<MobilityModel> ();
		Vector position = edMobility->GetPosition ();
		m_sites.push_back(position);

		MinX = min (MinX, position.x);
		MaxX = max (MaxX, position.x);
		MinY = min (MinY, position.y);
		MaxY = max (MaxY, position.y);
		MinZ = min (MinZ, position.z);
		MaxZ = max (MaxZ, position.z);
		//std::cout<<position<<"\n";
	}
	m_bounds = {
			{ MinX, MinY, MinZ },
			{ MaxX, MaxY, MaxZ }
	};
}

void
UDCPositionAllocator::CoverSites ( double radius )
{
  m_radius = radius;
  FastUDC(radius);
}
void
UDCPositionAllocator::FastUDC (double radius) {
	// see https://link.springer.com/content/pdf/10.1007%2F978-3-030-34029-2_10.pdf
    std::unordered_set< std::pair<int,int>, pair_hash> hashTableForLatticeDiskCenters;
    const double sqrt2 = std::sqrt(2);
    const double gridWidth = sqrt2 * radius;
    const double additiveFactor = gridWidth/2;
  //  const double onePointFiveOverSqrt2 =  1.5/sqrt2;
//    const double oneOverTwoSqrt2 = 1/(2*sqrt2);
    const double gridWidthTimesOnePointFive = gridWidth * 1.5;
    const double gridWidthTimesZeroPointFive = gridWidth * 0.5;

    double verticalTimesGridWidth, horizontalTimesGridWidth;
    int vertical, horizontal;

    for( Vector p : m_sites ) {
    	//std::cout<<p<<"\n";
        vertical = floor(p.x/gridWidth);
        horizontal = floor(p.y/gridWidth);
        verticalTimesGridWidth  = vertical * gridWidth;
        horizontalTimesGridWidth  = horizontal * gridWidth;

        if( isPresent(hashTableForLatticeDiskCenters,vertical,horizontal))
           continue;

        if( p.x >= verticalTimesGridWidth + gridWidthTimesOnePointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical+1, horizontal )
         && !(SquaredDistance( p, Vector(gridWidth*(vertical+1)+additiveFactor, horizontalTimesGridWidth+additiveFactor, 0) )>1) )
            continue;

        if( p.x <= verticalTimesGridWidth - gridWidthTimesZeroPointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical-1, horizontal )
         && !(SquaredDistance( p, Vector(gridWidth*(vertical-1)+additiveFactor, horizontalTimesGridWidth+additiveFactor, 0) )>1) )
            continue;

        if( p.y <= horizontalTimesGridWidth - gridWidthTimesZeroPointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical, horizontal-1 )
         && !(SquaredDistance( p, Vector(verticalTimesGridWidth+additiveFactor, gridWidth*(horizontal-1)+additiveFactor, 0) )>1) )
            continue;

        if( p.y >= horizontalTimesGridWidth + gridWidthTimesOnePointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical, horizontal+1 )
         && !(SquaredDistance( p, Vector(verticalTimesGridWidth+additiveFactor, gridWidth*(horizontal+1)+additiveFactor, 0) )>1) )
            continue;

        hashTableForLatticeDiskCenters.insert(std::pair<int,int>(vertical,horizontal));
        Add (Vector(verticalTimesGridWidth+additiveFactor, horizontalTimesGridWidth+additiveFactor, m_defaultHeight));
    }
}

void
UDCPositionAllocator::Print()
{
	using std::string;
	using std::max;
	using std::cout;
	using std::endl;

	string FileName = "temp";
	string TexFileName = FileName + ".tex";
    std::FILE *fp = fopen (TexFileName.c_str() ,"w");

    fprintf (fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf (fp,"\n\n\n\\begin{tikzpicture}\n\n");

    // Print origin
    //fprintf(fp,"\\draw [fill=white,stroke=green] (0,0) circle [radius=%f];\n",radiusOfPoints);

    const double ResultantDimension = 100;

    double ResizeFactor = ResultantDimension
    			 	 	     / max (m_bounds[1].x-m_bounds[0].x,
    						        max (m_bounds[1].y-m_bounds[0].y,
    								     m_bounds[1].z-m_bounds[0].z));

    const double RadiusOfPoints = 0.005*ResultantDimension;

    for(Vector p : m_sites)
        fprintf(fp, "\\draw [fill=black,stroke=black] (%f,%f) circle [radius=%f];\n",
        		p.x*ResizeFactor, p.y*ResizeFactor, RadiusOfPoints);

    for(Vector p : m_positions){
        //fprintf (fp, "\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",
    	//        p.x, p.y, RadiusOfPoints);
        fprintf (fp, "\\draw (%f,%f) circle [radius=%f];\n",
        		 p.x*ResizeFactor, p.y*ResizeFactor, m_radius*ResizeFactor);
    }

    fprintf (fp, "\n\n\\end{tikzpicture}");
    fprintf (fp, "\n\n\\end{document}");
    fclose (fp);

    cout << "\nOutput PDF generation started..." << endl;
    string command = "pdflatex " + TexFileName + " > /dev/null";
    system (command.c_str());
    cout << "PDF generation terminated..." << endl;

    command = "atril " + FileName + ".pdf &";
    system (command.c_str());
}

void
UDCPositionAllocator::Add (Vector v)
{
  m_positions.push_back (v);
  m_current = m_positions.begin ();
}

Vector
UDCPositionAllocator::GetNext (void) const
{
  Vector v = *m_current;
  m_current++;
  if (m_current == m_positions.end ())
    {
      m_current = m_positions.begin ();
    }
  return v;
}
int64_t
UDCPositionAllocator::AssignStreams (int64_t stream)
{
  return 0;
}

uint32_t
UDCPositionAllocator::GetSize (void) const
{
  return m_positions.size ();
}

uint32_t
UDCPositionAllocator::GetSitesN (void) const
{
  return m_sites.size ();
}


} // namespace ns3 
