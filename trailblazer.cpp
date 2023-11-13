// This is the CPP file you will edit and turn in.
// Also remove these comments here and add your own, along with
// comments on every function and on complex code sections.
// TODO: write comment header for this file; remove this comment

#include "trailblazer.h"
#include "queue.h"
#include "stack.h"
#include "hashmap.h"
#include "priorityqueue.h"
// TODO: include any other headers you need; remove this comment

using namespace std;

void depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex* end, Vector<Vertex*> visited, Vector<Vertex*>& path, bool& endFound) {
   if(!visited.contains(start)) {
       visited.add(start);
       start->setColor(GREEN);
       for(Vertex* neighbor : graph.getNeighbors(start)) {
           if(neighbor == end) {
               visited.add(neighbor);
               start->setColor(GREEN);
               path = visited;
               endFound = true;
               break;
           }
           depthFirstSearchHelper(graph, neighbor, end, visited, path, endFound);
           if(endFound) {
               end->setColor(GREEN);
               break;
           }
       }
   }
}

Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
   Vector<Vertex*> visited, path;
   bool endFound = false;
   if(start == end) path.add(start);
   else depthFirstSearchHelper(graph, start, end, visited, path, endFound);
   return path;
}

Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
   bool endFound = false;
   Queue<Vertex*> searchQueue;
   searchQueue.enqueue(start);
   Vector<Vertex*> visited;
   HashMap<Vertex*, Vertex*> mapOfPrevious;

   // Conducts a breadth first search by enqueueing and dequeueing a vertex pointer
   // and its neighboring vertexes until the endpoint is found
   while(!searchQueue.isEmpty()) {
       Vertex* v = searchQueue.dequeue();
       visited.add(v);
       v->setColor(GREEN);
       if(v == end) {
           endFound = true;
           break;
       }
       for(Vertex* neighbor : graph.getNeighbors(v)) {
           if(!visited.contains(neighbor)) {
               mapOfPrevious.add(neighbor, v);
               searchQueue.enqueue(neighbor);
               neighbor->setColor(YELLOW);
           }
       }
   }

   // Tracks path backward using the hashmap of vertexes and its previous vertex then
   // pushes each vertex into a stack, then reverses the order into a vector to return
   Vector<Vertex*> path;
   if(endFound) {
       Stack<Vertex*> pathTracker;
       Vertex* previous = end;
       while(previous != start) {
           pathTracker.push(previous);
           previous = mapOfPrevious.get(previous);
       }
       path.add(start);
       while(!pathTracker.isEmpty())
           path.add(pathTracker.pop());
   }
   return path;
}

Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
   PriorityQueue<Vertex*> searchQueue;
   searchQueue.add(start, 1);
   Vector<Vertex*> visited;
   HashMap<Vertex*, Vertex*> mapOfPrevious;
   HashMap<Vertex*, double> vertexCosts;
   vertexCosts.add(start, 0);

   // Searches for the best path from vertex to vertex based on lowest cost
   // using dijkstra's algorithm by enqueueing vertex's to a priority queue
   // prioritized based on cost from start to current vertex
   while(!searchQueue.isEmpty()) {
       Vertex* v = searchQueue.dequeue();
       v->setColor(GREEN);
       if(v == end) break;
       double vCost = vertexCosts.get(v);

       // Checks each neighboring vertex & its cost to travel there; if a
       // lower cost is found to travel to a vertex, that cost is updated
       for(Vertex* neighbor : graph.getNeighbors(v)) {
           Edge* e = graph.getEdge(v, neighbor);
           double nCost = e->cost + vCost;
           if(visited.contains(neighbor)) {
              if(vertexCosts.get(neighbor) > nCost) {
                  vertexCosts.put(neighbor, nCost);
                  mapOfPrevious.add(neighbor, v);
                  searchQueue.changePriority(neighbor, nCost);
              }
           } else {
               vertexCosts.put(neighbor, nCost);
               mapOfPrevious.add(neighbor, v);
               searchQueue.enqueue(neighbor, nCost);
               visited.add(neighbor);
               neighbor->setColor(YELLOW);
           }
       }
   }

   // Tracks path backward using the hashmap of vertexes and its previous vertex then
   // pushes each vertex into a stack, then reverses the order into a vector to return
   Stack<Vertex*> pathTracker;
   Vertex* previous = end;
   while(previous != start) {
       pathTracker.push(previous);
       previous = mapOfPrevious.get(previous);
   }
   Vector<Vertex*> path;
   path.add(start);
   while(!pathTracker.isEmpty())
       path.add(pathTracker.pop());
   return path;
}

Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
   PriorityQueue<Vertex*> searchQueue;
   searchQueue.add(start, 1);
   Vector<Vertex*> visited;
   HashMap<Vertex*, Vertex*> mapOfPrevious;
   HashMap<Vertex*, double> vertexCosts;
   vertexCosts.add(start, heuristicFunction(start, end));

   // Searches for the best path from vertex to vertex based on lowest cost
   // using the a-star algorithm by enqueueing vertex's to a priority queue
   // prioritized based on cost + heuristic from start to current vertex
   while(!searchQueue.isEmpty()) {
       Vertex* v = searchQueue.dequeue();
       v->setColor(GREEN);
       if(v == end) break;
       double vCost = vertexCosts.get(v);

       // Checks each neighboring vertex & its cost + heuristic to travel there; if a
       // lower cost is found to travel to a vertex, that cost is updated
       for(Vertex* neighbor : graph.getNeighbors(v)) {
           Edge* e = graph.getEdge(v, neighbor);
           double nCost = e->cost + vCost;
           if(visited.contains(neighbor)) {
              if(vertexCosts.get(neighbor) > nCost) {
                  vertexCosts.add(neighbor, nCost);
                  mapOfPrevious.add(neighbor, v);
                  searchQueue.changePriority(neighbor, nCost + heuristicFunction(neighbor, end));
              }
           } else {
               vertexCosts.add(neighbor, nCost);
               mapOfPrevious.add(neighbor, v);
               searchQueue.enqueue(neighbor, nCost + heuristicFunction(neighbor, end));
               visited.add(neighbor);
               neighbor->setColor(YELLOW);
           }
       }
   }

   // Tracks path backward using the hashmap of vertexes and its previous vertex then
   // pushes each vertex into a stack, then reverses the order into a vector to return
   Stack<Vertex*> pathTracker;
   Vertex* previous = end;
   while(previous != start) {
       pathTracker.push(previous);
       previous = mapOfPrevious.get(previous);
   }
   Vector<Vertex*> path;
   path.add(start);
   while(!pathTracker.isEmpty())
       path.add(pathTracker.pop());
   return path;
}

bool combineClusters(Edge* e, Set<Set<Vertex*>>& vertexClusters) {
   bool startFound = false;
   bool endFound = false;
   Set<Vertex*> clusterToAdd;

   // Searches for a cluster that has either vertex attached to the edge being checked
   // If a cluster contains both vertexes, then vertexes are already connected
   for(Set<Vertex*> cluster : vertexClusters) {
       if(cluster.contains(e->start) && cluster.contains(e->end))
           return false; // vertexes already connected since they're in the same cluster
       if(cluster.contains(e->start)) {
           clusterToAdd = cluster;
           vertexClusters.remove(cluster);
           startFound = true;
           break;
       }
       if(cluster.contains(e->end)) {
           clusterToAdd = cluster;
           vertexClusters.remove(cluster);
           endFound = true;
           break;
       }
   }

   // If neither of the vertexes were found in any of the clusters,
   // then a new cluster is made and added into the set of clusters
   if(!startFound && !endFound) {
       clusterToAdd.add(e->start);
       clusterToAdd.add(e->end);
       vertexClusters.add(clusterToAdd);
       return true;
   }

   // Searches for a cluster that has the opposite vertex, then adds the
   // contents of that opposite cluster into the previously found cluster
   for(Set<Vertex*> cluster : vertexClusters) {
       if(startFound && cluster.contains(e->end)) {
           vertexClusters.remove(cluster);
           for(Vertex* v : clusterToAdd) cluster.add(v);
           vertexClusters.add(cluster);
           return true;
       }
       if(endFound && cluster.contains(e->start)) {
           vertexClusters.remove(cluster);
           for(Vertex* v : clusterToAdd) cluster.add(v);
           vertexClusters.add(cluster);
           return true;
       }
   }

   // If only one vertex is found in a cluster,
   // then the other vertex will be added to that cluster
   if(startFound) clusterToAdd.add(e->end);
   if(endFound) clusterToAdd.add(e->start);
   vertexClusters.add(clusterToAdd);
   return true;
}

Set<Edge*> kruskal(BasicGraph& graph) {
   Set<Edge*> mst;
   Set<Set<Vertex*>> vertexClusters;
   PriorityQueue<Edge*> edgeQueue;
   for(Edge* e : graph.getEdgeSet())
       edgeQueue.add(e, e->cost);
   while(!edgeQueue.isEmpty()) {
       Edge* e = edgeQueue.dequeue();
       if(combineClusters(e, vertexClusters)) mst.add(e);
   }
   return mst;
}
