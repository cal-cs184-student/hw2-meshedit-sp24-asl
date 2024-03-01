#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (class member).
     *
     * @param points A vector of points in 2D
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const& points)
    {
        // TODO Part 1.
        std::vector<Vector2D> intermediatePoints;

        // Loop through each pair of consecutive points
        for (size_t i = 0; i < points.size() - 1; ++i) {
            // Linear interpolation between points[i] and points[i + 1]
            Vector2D interpolatedPoint = (1 - t) * points[i] + t * points[i + 1];

            // Add the computed point to the intermediate points vector
            intermediatePoints.push_back(interpolatedPoint);
        }

        return intermediatePoints;
    }

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (function parameter).
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const& points, double t) const
    {
        // TODO Part 2.
        std::vector<Vector3D> intermediatePoints;

        // Iterate over each pair of adjacent points and apply the linear interpolation formula
        for (size_t i = 0; i < points.size() - 1; ++i) {
            Vector3D interpolatedPoint = (1 - t) * points[i] + t * points[i + 1];
            intermediatePoints.push_back(interpolatedPoint);
        }

        return intermediatePoints;
    }

    /**
     * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const& points, double t) const
    {
        // TODO Part 2.
        // Base case: if there's only one point left, return it
        if (points.size() == 1) {
            return points[0];
        }

        // Recursive step: apply one step of the de Casteljau algorithm and then recurse
        std::vector<Vector3D> intermediatePoints = evaluateStep(points, t);
        return evaluate1D(intermediatePoints, t); // Recurse with the new set of points
    }

    /**
     * Evaluates the Bezier patch at parameter (u, v)
     *
     * @param u         Scalar interpolation parameter
     * @param v         Scalar interpolation parameter (along the other axis)
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate(double u, double v) const
    {
        // TODO Part 2.
        std::vector<Vector3D> curvePoints;

        // Evaluate each row of control points at u to generate points on the intermediate curves
        for (const auto& row : controlPoints) {
            Vector3D pointOnCurve = evaluate1D(row, u);
            curvePoints.push_back(pointOnCurve);
        }

        // Now evaluate the curve defined by curvePoints at v
        return evaluate1D(curvePoints, v);
    }

    Vector3D Vertex::normal(void) const
    {
        // TODO Part 3.
        // Returns an approximate unit normal at this vertex, computed by
        // taking the area-weighted average of the normals of neighboring
        // triangles, then normalizing.
        Vector3D n(0, 0, 0); // Initialize the normal to be (0,0,0)

        // Start from one of the halfedges emanating from the vertex
        HalfedgeCIter h = halfedge();
        // Make sure to start from the beginning of the loop
        const HalfedgeCIter hBegin = h;

        // Iterate over all incident faces
        do {
            // Get the current face
            FaceCIter f = h->face();

            // Compute the normal for this face
            Vector3D a = h->next()->vertex()->position - h->vertex()->position;
            Vector3D b = h->next()->next()->vertex()->position - h->next()->vertex()->position;
            Vector3D faceNormal = cross(a, b);

            // Accumulate the area-weighted face normal
            n += faceNormal;

            // Move to the next halfedge around the vertex
            h = h->twin()->next();
        } while (h != hBegin); // Make sure not to loop infinitely

        // Normalize the accumulated normal to get the unit normal
        if (n.norm() > 0) {
            n.normalize();
        }

        return n;
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
    {
        // TODO Part 4.
        // This method should flip the given edge and return an iterator to the flipped edge.
        if (e0->isBoundary()) {
            return e0; // Cannot flip a boundary edge
        }

        // Step 2: Identify affected elements
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        // Step 3: Reassign pointers
        // Update halfedges
        h0->setNeighbors(h1, h3, v3, e0, f0); //next,twin,vertex,edge,face
        h1->setNeighbors(h2, h7, v2, e2, f0);
        h2->setNeighbors(h0, h8, v0, e3, f0);
        h3->setNeighbors(h4, h0, v2, e0, f1);
        h4->setNeighbors(h5, h9, v3, e4, f1);
        h5->setNeighbors(h3, h6, v1, e1, f1);
        h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
        h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
        h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
        h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

        // Update vertices
        v0->halfedge() = h2;
        v1->halfedge() = h5;
        v2->halfedge() = h3;//h3
        v3->halfedge() = h0;//h0

        // Update vertices
        e0->halfedge() = h0;
        e1->halfedge() = h5;
        e2->halfedge() = h1;
        e3->halfedge() = h2;
        e4->halfedge() = h4;

        // Update faces
        f0->halfedge() = h0;
        f1->halfedge() = h3;

        // Return the flipped edge
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
    {
        // TODO Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        if (e0->isBoundary()) {
            return VertexIter(); // Cannot flip a boundary edge
        }

        // Step 2: Identify affected elements
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();

        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        // Compute the midpoint of the original edge
        Vector3D midpoint = 0.5f * v3->position + 0.5f * v2->position;

        // Create a new vertex at the midpoint
        VertexIter vm = newVertex();
        vm->position = midpoint;

        // Assign the new half-edges to the new edges
        HalfedgeIter hn0 = newHalfedge();
        HalfedgeIter hn1 = newHalfedge();
        HalfedgeIter hn2 = newHalfedge();
        HalfedgeIter hn3 = newHalfedge();
        HalfedgeIter hn4 = newHalfedge();
        HalfedgeIter hn5 = newHalfedge();


        EdgeIter en0 = newEdge();
        EdgeIter en1 = newEdge();
        EdgeIter en2 = newEdge();

        FaceIter fn0 = newFace();
        FaceIter fn1 = newFace();

        h0->setNeighbors(h1, h3, vm, e0, fn1);
        h1->setNeighbors(h2, h6, v1, e1, fn1);
        h2->setNeighbors(h0, hn0, v2, en2, fn1);
        h3->setNeighbors(h4, h0, v1, e0, fn0);
        h4->setNeighbors(h5, hn5, vm, en1, fn0);
        h5->setNeighbors(h3, h9, v3, e4, fn0);
        h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
        h7->setNeighbors(h7->next(), hn1, v0, e2, h7->face());
        h8->setNeighbors(h8->next(), hn4, v3, e3, h8->face());
        h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
        hn0->setNeighbors(hn1, h2, vm, en2, f0);
        hn1->setNeighbors(hn2, h7, v2, e2, f0);
        hn2->setNeighbors(hn0, hn3, v0, en0, f0);
        hn3->setNeighbors(hn4, hn2, vm, en0, f1);
        hn4->setNeighbors(hn5, h8, v0, e3, f1);
        hn5->setNeighbors(hn3, h4, v3, en1, f1);

        // Update vertices
        v0->halfedge() = hn2; //hn0
        v1->halfedge() = h3; //h
        v2->halfedge() = h2;//h2
        v3->halfedge() = hn5;//h5
        vm->halfedge() = h0;

        // Update vertices
        e0->halfedge() = h0;
        e1->halfedge() = h1;
        e2->halfedge() = h7;
        e3->halfedge() = h8;
        e4->halfedge() = h5;
        en0->halfedge() = hn2;
        en1->halfedge() = h4;
        en2->halfedge() = h2; //hn4

        // Update faces
        f0->halfedge() = hn0;
        f1->halfedge() = hn3;
        fn0->halfedge() = h4;
        fn1->halfedge() = h1;
        return vm;
    }


    void MeshResampler::upsample(HalfedgeMesh& mesh) {
        // Step 1: Compute new positions for original vertices and mark them
        VertexIter v = mesh.verticesBegin();
        while (v != mesh.verticesEnd()) {

            // get the next edge NOW!
            VertexIter nextVert = v;
            nextVert++; 
            Vector3D sumOfAdjVertices(0, 0, 0);
            int n = 0; // Vertex valence

            HalfedgeCIter start = v->halfedge();
            start = start->twin();
            HalfedgeCIter current = start;

            do {
                sumOfAdjVertices += current->vertex()->position; // Summing up adjacent vertex positions
                current = current->next()->twin();
                n++; // Counting the number of adjacent vertices, which is the valence
            } while (current != start);

            float u = (n == 3) ? 3.0f / 16 : 3.0f / (8 * n);
            v->newPosition = (1 - n * u) * v->position + u * sumOfAdjVertices;
            v->isNew = false; // Mark as original vertex
            v = nextVert;
        }

        // Step 2: Compute new positions for edge vertices
        EdgeIter e = mesh.edgesBegin();
        std::vector<EdgeIter> originalEdges;
        while (e != mesh.edgesEnd()) {

            // get the next edge NOW!
            EdgeIter nextEdge = e;
            nextEdge++;
            Vector3D p1 = e->halfedge()->vertex()->position;
            Vector3D p2 = e->halfedge()->twin()->vertex()->position;
            Vector3D p3 = e->halfedge()->next()->next()->vertex()->position; // Opposite vertex 1
            Vector3D p4 = e->halfedge()->twin()->next()->next()->vertex()->position; // Opposite vertex 2
            e->newPosition = 0.375 * (p1 + p2) + 0.125 * (p3 + p4);
            originalEdges.push_back(e);
            e = nextEdge;
        }

        // Step 3: Split every original edge
        for (EdgeIter e : originalEdges) {
            // Retrieve the pre-calculated position for the midpoint of the current edge
            Vector3D midpointPosition = e->newPosition;

            // Split the current edge to create a new vertex at its midpoint
            VertexIter midpointVertex = mesh.splitEdge(e);

            // Mark the newly created vertex as a new vertex
            midpointVertex->isNew = true;

            // Set the position of the newly created vertex to the pre-calculated midpoint position
            midpointVertex->newPosition = midpointPosition;

            // Iterate over the edges connected to the new vertex to set their 'isNew' status
            HalfedgeIter halfEdge = midpointVertex->halfedge();
            for (int j = 0; j < 4; ++j) { // A vertex in a triangular mesh typically connects to 3 or 4 edges
                // Mark every other edge as new, starting with the second one encountered
                halfEdge->edge()->isNew = (j % 2 == 1);

                // Move to the next edge around the vertex
                halfEdge = halfEdge->twin()->next();
            }
        }
        
        // Step 4: Flip new edges
        e = mesh.edgesBegin();
        while (e != mesh.edgesEnd()) {

            // get the next edge NOW!
            EdgeIter nextEdge = e;
            nextEdge++;
            if (e->isNew && (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew)) {
                mesh.flipEdge(e);
            }
            e = nextEdge;
        }

        
        // Step 5: Update vertex positions
        v = mesh.verticesBegin();
        while (v != mesh.verticesEnd()) {

            // get the next edge NOW!
            VertexIter nextVert = v;
            nextVert++;
            v->position = v->newPosition;
            v = nextVert;
        }
        return;
    }
}