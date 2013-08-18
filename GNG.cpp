#include "GNG.h"

#include <iostream>
#include <fstream>
#include <limits>
#include <boost/foreach.hpp>
#include <boost/utility.hpp> 
#include <boost/graph/graphviz.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace ng;
using namespace boost::numeric;

GNG::GNG(int dim, double eps_w, double eps_n, double alpha, double beta, int lambda, int age_max, int vertices_max, double k, bool gngu): dim(dim), eps_w(eps_w), eps_n(eps_n), alpha(alpha), beta(beta), lambda(lambda), iteration_count(0), vertices_max(vertices_max), age_max(age_max), k(k), gngu(gngu), class_count(0)
{
}

void GNG::init(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2)
{
	GNGVertex vertex_v1 = boost::add_vertex(graph);
	graph[vertex_v1].weight = v1;
	graph[vertex_v1].error = 0.;
	graph[vertex_v1].utility = 0.;
	GNGVertex vertex_v2 = boost::add_vertex(graph);
	graph[vertex_v2].weight = v2;
	graph[vertex_v2].error = 0.;
	graph[vertex_v2].utility = 0.;
	GNGEdge edge_v1_v2 = boost::add_edge(vertex_v1, vertex_v2, graph).first;
	graph[edge_v1_v2].age = 0;
}

void GNG::addSignal(const boost::numeric::ublas::vector<double> &x)
{
	//std::cout<<"find two nearest nodes for x\n";
	GNGVertexIterator s, t;
	double error_max = std::numeric_limits<double>::min();
	double dist_s = std::numeric_limits<double>::max();
	double dist_t = std::numeric_limits<double>::max();

	GNGVertexIterator vertex_begin, vertex_end;
	boost::tie(vertex_begin, vertex_end) = boost::vertices(graph);
	for(GNGVertexIterator i = vertex_begin; i != vertex_end; i++)
	{
		double dist = ublas::norm_2(x-graph[*i].weight);
		if(dist < dist_s)
		{
			t = s;
			dist_t = dist_s;
			s = i;
			dist_s = dist;
		}
		else if(dist < dist_t)
		{
			t = i;
			dist_t = dist;
		}
		if(dist > error_max)
		{
			error_max = dist;
		}
	}
	//std::cout<<"update error, usefulness and weight for s\n";
	graph[*s].error += dist_s;
	graph[*s].utility += (dist_t - dist_s);
	graph[*s].weight -= eps_w * (graph[*s].weight - x);
	//std::cout<<"update the weight of adjacent vertices\n";
	GNGAdjacencyIterator adjacency_begin, adjacency_end;
	boost::tie(adjacency_begin, adjacency_end) = boost::adjacent_vertices(*s, graph);
	for(GNGAdjacencyIterator i = adjacency_begin; i != adjacency_end; i++)
	{
		graph[*i].weight -= eps_n * (graph[*i].weight - x);
	}
	//std::cout<<"update the age of adjacent vertices\n";
	GNGOutEdgeIterator edge_out_begin, edge_out_end;
	boost::tie(edge_out_begin, edge_out_end) = boost::out_edges(*s, graph);
	for(GNGOutEdgeIterator i = edge_out_begin; i != edge_out_end; i++)
	{
		graph[*i].age ++;
	}
	//std::cout<<"create edge s<->t or set to zero it edge\n";
	std::pair<GNGEdge, bool> e = boost::edge(*s, *t, graph);
	if(e.second)
	{
		graph[e.first].age = 0;
	}
	else
	{
		GNGEdge tmp = boost::add_edge(*s, *t, graph).first;
		graph[tmp].age = 0;
	}
	//std::cout<<"delete all edges whose age > age_max\n";
	GNGEdgeIterator edge_begin, edge_end;
	boost::tie(edge_begin, edge_end) = boost::edges(graph);
	GNGEdgeIterator edge_next;
	for(GNGEdgeIterator i = edge_next = edge_begin; i != edge_end; i = edge_next)
	{
		edge_next ++;
		if(graph[*i].age > age_max)
		{
			GNGVertex vertex_s = boost::source(*i, graph);
			GNGVertex vertex_t = boost::target(*i, graph);
			boost::remove_edge(vertex_s, vertex_t, graph);
			if(!gngu)
			{
				if(!boost::out_degree(vertex_s, graph))
				{
					boost::remove_vertex(vertex_s, graph);
				}
				if(!boost::out_degree(vertex_t, graph))
				{
					boost::remove_vertex(vertex_t, graph);
				}
			}
		}
	}
	//std::cout<<"delete vertex min_utility\n";
	if(gngu)
	{
		boost::tie(vertex_begin, vertex_end) = boost::vertices(graph);
		double utility_min = std::numeric_limits<double>::max();
		GNGVertexIterator vertex_utility_min = vertex_begin;
		for(GNGVertexIterator i = vertex_begin; i != vertex_end; ++i)
		{
			if(utility_min > graph[*i].utility)
			{
				vertex_utility_min = i;
				utility_min = graph[*i].utility;
			}
		}
		if(boost::num_vertices(graph) > 2 && utility_min > 1e-07 && error_max / utility_min > k)
		{
			boost::clear_vertex(*vertex_utility_min, graph);
			boost::remove_vertex(*vertex_utility_min, graph);
		}
	}
	//std::cout<<"may be add vertex?\n";
	if((iteration_count % lambda == 0) && (boost::num_vertices(graph) < vertices_max))
	{
		GNGVertex u;
		GNGVertex v;
		double u_error = -1;
		double v_error = -1;
		boost::tie(vertex_begin, vertex_end) = boost::vertices(graph);
		for(GNGVertexIterator i = vertex_begin; i != vertex_end; i++)
		{
			if(graph[*i].error > u_error)
			{
				u_error = graph[*i].error;
				u = *i;
			}
		}
		v = u;
		boost::tie(adjacency_begin, adjacency_end) = boost::adjacent_vertices(u, graph);
		for(GNGAdjacencyIterator i = adjacency_begin; i != adjacency_end; i++)
		{
			double err = graph[*i].error;
			if(graph[*i].error > v_error)
			{
				v_error = graph[*i].error;
				v = *i;
			}
		}
		GNGVertex r = boost::add_vertex(graph);
		graph[r].weight = (graph[u].weight + graph[v].weight) / 2.;
		GNGEdge e1 = boost::add_edge(u, r, graph).first;
		GNGEdge e2 = boost::add_edge(r, v, graph).first;
		boost::remove_edge(u, v, graph);
		graph[e1].age = 0;
		graph[e2].age = 0;
		graph[u].error *= alpha;
		graph[v].error *= alpha;
		graph[r].error = graph[u].error;
		graph[r].utility = (graph[u].utility + graph[v].utility) / 2.;
		iteration_count = 0;
	}
	//std::cout<<"reduce the error of all neurons\n";
	boost::tie(vertex_begin, vertex_end) = boost::vertices(graph);
	for(GNGVertexIterator i = vertex_begin; i != vertex_end; i++)
	{
		graph[*i].error *= (1-beta);
		graph[*i].utility *= (1-beta);
	}
	iteration_count++;
}

int GNG::getConnectedComponentsCount()
{
	size_t index = 0;
	BGL_FORALL_VERTICES(v, graph, GNGGraph)
	{
		boost::put(boost::vertex_index, graph, v, index++);
	}
	GNGComponentMap component;
	boost::associative_property_map<GNGComponentMap> component_map(component);
	int com_count = connected_components(graph, component_map);
	return com_count;
}

GNGGraph GNG::getGraph()
{
	return graph;
}

void GNG::classify()
{
	if(!class_count)
	{
		size_t index = 0;
		BGL_FORALL_VERTICES(v, graph, GNGGraph)
		{
			boost::put(boost::vertex_index, graph, v, index++);
		}
		GNGComponentMap component;
		boost::associative_property_map<GNGComponentMap> component_map(component);
		class_count = connected_components(graph, component_map);
		BGL_FORALL_VERTICES(v, graph, GNGGraph)
		{
			graph[v].class_id = boost::get(component_map, v);
		}
	}
	else
	{
		size_t index = 0;
		BGL_FORALL_VERTICES(v, graph, GNGGraph)
		{
			boost::put(boost::vertex_index, graph, v, index++);
		}
		GNGComponentMap component;
		boost::associative_property_map<GNGComponentMap> component_map(component);
		int class_num = connected_components(graph, component_map);
		std::map<int, int> label_map;
		BGL_FORALL_VERTICES(v, graph, GNGGraph)
		{
			if(graph[v].class_id != -1)
			{
				label_map[boost::get(component_map, v)] = graph[v].class_id;
			}
		}
		if(class_count < class_num)
		{
			for(int i = class_count; i < class_num; i++)
			{
				label_map[i] = class_count++;
			}
		}
		else
		{
			class_count = class_num;
		}
		BGL_FORALL_VERTICES(v, graph, GNGGraph)
		{
			graph[v].class_id = label_map[boost::get(component_map, v)];
		}
	}
}

void GNG::draw(cv::Mat &image)
{
	const cv::Scalar colors[6] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255)};
	GNGEdgeIterator edge_begin, edge_end;
	boost::tie(edge_begin, edge_end) = boost::edges(graph);
	for(GNGEdgeIterator i = edge_begin; i != edge_end; i++)
	{
		GNGVertex vertex_s = boost::source(*i, graph);
		GNGVertex vertex_t = boost::target(*i, graph);
		cv::Point p1(graph[vertex_s].weight[0], graph[vertex_s].weight[1]);
		cv::Point p2(graph[vertex_t].weight[0], graph[vertex_t].weight[1]);
		cv::line(image, p1, p2, colors[graph[vertex_s].class_id%6]);
	}
	GNGVertexIterator vertex_begin, vertex_end;
	boost::tie(vertex_begin, vertex_end) = boost::vertices(graph);
	for(GNGVertexIterator i = vertex_begin; i != vertex_end; i++)
	{
		cv::circle(image, cv::Point(graph[*i].weight[0], graph[*i].weight[1]), 3, colors[graph[*i].class_id%6]);
	}
}