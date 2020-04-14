using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    
    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private HashSet<GameObject> collidedNeighbors = new HashSet<GameObject>();
    
    
    public void Start()
    {
        Debug.unityLogger.logEnabled = false;
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
        
    }

    public void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (true)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;
        
        // Comment below for Growing Spiral
        force += CalculateGoalForce() * 3;
        
        foreach (var obj in perceivedNeighbors)
        {
            if (AgentManager.IsAgent(obj))
            {
                force += CalculateAgentForce(obj)*0.000001f;
            }
            else
            {
                if (obj.gameObject.name == "Plane") continue;
                force += CalculateWallForce(obj)*0.0001f;
            }
        }
        // Comment until here

            
        // Uncomment below for growing Spiral
        // force = GrowingSpiralForce();
        
        if (force != Vector3.zero)
        {
            force.y = 0;
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }

    private Vector3 GrowingSpiralForce()
    {
        var spiralForce = Vector3.zero;
        var centerDirection = Vector3.zero - transform.position;
        if (centerDirection.magnitude > 0)
        {
            spiralForce += Vector3.Cross(Vector3.up, centerDirection).normalized * 0.01f;
            spiralForce += centerDirection.normalized * 0.005f;
        }

        return spiralForce;
    }
    protected virtual Vector3 CalculateGoalForce()
    {
       if (path.Count == 0)
       {
            return Vector3.zero;
       }
       var desiredVel = (path[0] - transform.position);
        
       desiredVel = desiredVel.normalized * Mathf.Min(desiredVel.magnitude, Parameters.maxSpeed);
       var calculateAcc = (desiredVel - this.GetVelocity())/Parameters.T;
       
       return mass*calculateAcc;
       
    }

    private Vector3 CalculateAgentForce(GameObject agt)
    {
        var agentForce = Vector3.zero;
        var direction = (transform.position - agt.transform.position);
        //Pyschological Force
        var sumRadii = radius + agt.GetComponent<NavMeshAgent>().radius;
        
        var com = Vector3.Distance(transform.position , agt.transform.position);
        var overflow = sumRadii - com;
        var exp = Mathf.Exp(overflow  / Parameters.B);
        agentForce += (Parameters.A * exp) * (direction.normalized);
        
        //Penetration Force
        if (collidedNeighbors.Contains(agt))
        {
            // agentForce += (Parameters.k) * (overflow) * (direction.normalized);
            agentForce += (Parameters.k) * overflow  * (direction.normalized) * 0.001f;
            Debug.Log("Pen Force");
        }

        //Sliding forces to add
        var tangent = Vector3.Cross(Vector3.up, direction.normalized);
        agentForce+=Parameters.Kappa*(overflow )*Vector3.Dot((rb.velocity-agt.GetComponent<Rigidbody>().velocity),tangent)*tangent;
        return agentForce;

    }

    private Vector3 CalculateWallForce(GameObject wall)
    {
        var wallForce = Vector3.zero;

        var normal = transform.position - wall.transform.position;
        normal.y = 0;
        if (Mathf.Abs(normal.x) > Mathf.Abs(normal.z))
        {
            normal.z = 0;
        }
        else
        {
            normal.x = 0;
        }
        normal = normal.normalized;

        var dir = transform.position - wall.transform.position;
        dir.y = 0;
        var projection = Vector3.Project(dir, normal);

        //var com = (rb.centerOfMass - wall.transform.position).magnitude;
        var exponent = Mathf.Exp(((radius+0.5f) - projection.magnitude) / Parameters.WALL_B);
        wallForce += (Parameters.WALL_A * exponent) * normal; //Direction?

        // penetration force, same as before.
       if (collidedNeighbors.Contains(wall))
        {
            Debug.Log("Collided with wall");
            if (((radius + 0.5f) - projection.magnitude) > 0)
            {
                /*var pt = wall.GetComponent<Collision>().contacts[0];
                var dir = pt.normal;
                dir.y=0;
                dir.normalized;*/
                wallForce += (Parameters.WALL_k * ((radius + 0.5f) - projection.magnitude)) * normal;


                //sliding force
                var tangent = Vector3.Cross(Vector3.up, dir);
                wallForce -= Parameters.WALL_Kappa * ((radius + 0.5f) - projection.magnitude) * Vector3.Dot(rb.velocity, tangent) * tangent;
            }
        }
        //Debug.Log("Wall" + 0.01f*wallForce);
        return wallForce;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;
        
        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
       
       perceivedNeighbors.Add(other.gameObject);
       
    }
    
    public void OnTriggerExit(Collider other)
    {
        if (perceivedNeighbors.Contains(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        collidedNeighbors.Add(collision.gameObject);
    }

    public void OnCollisionExit(Collision collision)
    {
        if (collidedNeighbors.Contains(collision.gameObject))
            collidedNeighbors.Remove(collision.gameObject);
    }

    #endregion
}
