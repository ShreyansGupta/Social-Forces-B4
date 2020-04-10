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

    private string wallDir = "back";
    
    public void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
        
        // Only for wall follower
        transform.position = new Vector3(-9f, 1.0f, -11f);
        // path = new List<Vector3>();
        // path.Add(new Vector3(12.0f, 1f, 12.0f));
    }

    public void Update()
    {
        Debug.Log(path.Count);
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 1.5f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (false)
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
        force += CalculateGoalForce()*5.0f;

        foreach (var obj in perceivedNeighbors)
        {
            if (AgentManager.IsAgent(obj))
            {
                force += CalculateAgentForce(obj)*0.01f;
            }
            else
            {

                force += CalculateWallForce(obj);
            }
        }
        
        if (force != Vector3.zero)
        {
            force.y = 0;
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }

    protected virtual Vector3 CalculateGoalForce()
    {
       //  if (path.Count == 0)
       //  {
       return Vector3.zero;
       //  }
       //  var desiredVel = (path[0] - transform.position);
       //  
       // // var desiredVel = desiredVel.normalized * Mathf.Min(desiredDirect.magnitude, Parameters.maxSpeed);
       //  var calculateAcc = (desiredVel - this.GetVelocity())/Parameters.T;
       //
       //  return mass*calculateAcc;
       
    }

    private Vector3 CalculateAgentForce(GameObject agt)
    {
        
        var agentForce = Vector3.zero;
        var direction = (transform.position - agt.transform.position);
        //Pyschological Force
        var sumRadii = radius + agt.GetComponent<NavMeshAgent>().radius;
        var com = (this.rb.centerOfMass - agt.GetComponent<Rigidbody>().centerOfMass).magnitude;
        var exp = Mathf.Exp((sumRadii - com) / Parameters.B);
        agentForce += (Parameters.A * exp) * (direction.normalized);

        //Penetration Force
        if (collidedNeighbors.Contains(agt))
        {
            agentForce += (Parameters.k) * (sumRadii - com) * (direction.normalized);
        }

        //Sliding forces to add
        var tangent = Vector3.Cross(Vector3.up, direction.normalized);
        agentForce+=Parameters.Kappa*(sumRadii-com)*Vector3.Dot((rb.velocity-agt.GetComponent<Rigidbody>().velocity),tangent)*tangent;
        //Debug.Log("Agent" + 0.1f*agentForce);
        return agentForce;

        }

    private Vector3 CalculateWallForce(GameObject wall)
    {
        var wallForce = Vector3.zero;

        if (wall.name == "Plane")
        {
            return wallForce;
        }
        
        var pos = transform.position;
        pos.y = 0f;

        RaycastHit hit;
        
        var force = Vector3.zero;
        bool colliding = false;

        if (Physics.Raycast(pos, transform.right, out hit) && hit.transform.gameObject == wall) 
        {
            wallDir = "right";
            colliding = true;
            force = Vector3.Cross(Vector3.up, hit.normal);
        }
        else if (Physics.Raycast(pos, -transform.right, out hit) && hit.transform.gameObject == wall)
        {
            wallDir = "left";
            colliding = true;
            force = -Vector3.Cross(Vector3.up, hit.normal);

        }
        else if (Physics.Raycast(pos, transform.forward, out hit) && hit.transform.gameObject == wall)
        {
            // rotate either to right or left depending on if we have wall on left or right
            // i.e. let's rotate in the direction of already applied force and add that force in the forward direction
            // to get the ball rolling
            colliding = true;
            if (wallDir == "left")
            {
                force = -Vector3.Cross(Vector3.up, hit.normal);
                transform.rotation = Quaternion.FromToRotation(transform.forward, transform.right) * transform.rotation;
            }
            else if (wallDir == "right")
            {
                force = Vector3.Cross(Vector3.up, hit.normal);
                transform.rotation = Quaternion.FromToRotation(transform.forward, -transform.right) * transform.rotation;
            }
        }

        if (colliding)
        {
            Debug.DrawRay(hit.point, Vector3.up*5, Color.red);
            Debug.DrawRay(hit.point, hit.normal * 5, Color.yellow);
            Debug.DrawRay(pos, force * 10, Color.white);
        }
        wallForce += force;

        
        // Normal force
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
