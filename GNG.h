#ifndef __GNG_H__
#define __GNG_H__

#include <boost/graph/adjacency_list.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <string>
#include <opencv2/core/core.hpp>

namespace ng
{
	struct GNGVertexProperties
	{
		double utility;
		double error;
		boost::numeric::ublas::vector<double> weight;
		int class_id;
		GNGVertexProperties():class_id(-1){}
	};

	struct GNGEdgeProperties
	{
		int age;
	};

	typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, boost::property<boost::vertex_index_t, size_t, GNGVertexProperties>, GNGEdgeProperties> GNGGraph;
	typedef boost::graph_traits<GNGGraph>::vertex_descriptor GNGVertex;
	typedef boost::graph_traits<GNGGraph>::edge_descriptor GNGEdge;
	typedef boost::graph_traits<GNGGraph>::vertex_iterator GNGVertexIterator;
	typedef boost::graph_traits<GNGGraph>::edge_iterator GNGEdgeIterator;
	typedef boost::graph_traits<GNGGraph>::out_edge_iterator GNGOutEdgeIterator;
	typedef boost::graph_traits<GNGGraph>::adjacency_iterator GNGAdjacencyIterator;
	typedef boost::graph_traits<GNGGraph>::vertices_size_type GNGVerticesSizeType;
	typedef std::map<GNGVertex, int> GNGComponentMap;

	class GNG
	{
	public:
		GNG(int dim, double eps_w = 0.2, double eps_n = 0.0006, double alpha = 0.5, double beta = 0.0005, int lambda = 20, int age_max = 5, int vertices_max = 4, double k = 2, bool gngu = true);
		~GNG(){}
		void addSignal(const boost::numeric::ublas::vector<double> &x);
		void init(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2);
		void classify();
		int getConnectedComponentsCount();
		GNGGraph getGraph();
	private:
		int dim;
		double eps_w, eps_n;
		double alpha, beta;
		int lambda;
		int iteration_count;
		int vertices_max;
		int age_max;
		double k;
		bool gngu;
		GNGGraph graph;
		int class_count;
	};
}

#endif __GNG_H__