using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Leader : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    private float leaderDist=1.5f;
    private bool isCollidingLeader;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private HashSet<GameObject> collidedNeighbors = new HashSet<GameObject>();

    public void Start()
    {
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
        }
        else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 1.5f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
               /* gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);*/
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
        //Debug.Log("Compute Path Called");
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
        var leader = LeaderManager.leader;
        bool isLeader = int.Parse(name.Split(' ')[1]) == 0;
        if (isLeader)
        {
            if (path.Count>0 && Vector3.Distance(transform.position, path[0]) > 0.5f)
                force += CalculateGoalForce()*3.5f;
            else
            {
                this.rb.velocity = Vector3.zero;
            }
        }
        else
        {
            if (isCollidingLeader)
            {
                //force += CalculateLeaderRepulsionForce(leader) * 5;
                if (Mathf.Abs(transform.position.x)>Mathf.Abs(transform.position.z))
                    transform.position = transform.position + new Vector3(0,0,0.1f) ;
                else
                    transform.position = transform.position + new Vector3(0.1f,0,0);

                
            }
            if (Vector3.Distance(transform.position, leader.transform.position) > 3.0f)
            {
                force += CalculateLeaderForce(leader)*2;
                force = force * 0.1f;
                
            }
            else
            {
                rb.velocity = Vector3.zero;

            }
        }

        foreach(var obj in collidedNeighbors)
        {
            if (obj.transform.parent!=null && obj.transform.parent.name == "Walls")
            {
                force += CalculateWallForce(obj)*0.01f;
            }
        }
        if (force != Vector3.zero)
        {
            force.y = 0;
            return force.normalized * Mathf.Min(force.magnitude,3f);
        }
        else
        {
            return Vector3.zero;
        }
    }
    protected virtual Vector3 CalculateGoalForce()
    {
        if (path.Count == 0)
        {
            return Vector3.zero;
        }
        var desiredVel = (path[0] - transform.position);

        var calculateAcc = (desiredVel - this.GetVelocity()) / Parameters.T;
        //Debug.DrawRay(transform.position, calculateAcc, Color.yellow);
        return mass * calculateAcc;

    }

    /*private Vector3 CalculateLeaderRepulsionForce(GameObject leader)
    {
        var agentForce = Vector3.zero;
        var sumRadii = leader.GetComponent<Leader>().radius + this.radius;
        var com = (leader.transform.position - transform.position);
        var exp = Mathf.Exp((sumRadii - com.magnitude) / Parameters.B);
        agentForce += (exp) * (com.normalized);
        return agentForce;
    }*/
        private Vector3 CalculateLeaderForce(GameObject leader)
    {

        var agentForce = Vector3.zero;
        var direction = (transform.position - leader.transform.position).normalized;
        
        var leaderVel = leader.GetComponent<Leader>().GetVelocity();
        leaderVel = leaderVel.normalized;
        var aheadPos = leader.transform.position + leaderVel;
        
        var onWay = Vector3.Dot(leaderVel, direction);
        if (Vector3.Distance(aheadPos, transform.position) < 3f)
        {
            Debug.Log("On the way");
            if (Mathf.Abs(transform.position.x) > Mathf.Abs(transform.position.z))
                transform.position = transform.position + new Vector3(0, 0, 1.5f);
            else
                transform.position = transform.position + new Vector3(1.5f, 0, 0);
            
            //agentForce += (transform.position-aheadPos)*0.001f;
        }
        
        agentForce -= (direction);
        //agentForce += tangent * 0.01f;
          

        Debug.DrawRay(transform.position, agentForce, Color.red);
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

        var exponent = Mathf.Exp(((radius + 0.5f) - projection.magnitude) / Parameters.WALL_B);
        wallForce += (Parameters.WALL_A * exponent) * normal; //Direction?

        // penetration force, same as before.
        if (collidedNeighbors.Contains(wall))
        {
            Debug.Log("Collided with wall");
            if (((radius + 0.5f) - projection.magnitude) > 0)
            {
               
                wallForce += (Parameters.WALL_k * ((radius + 0.5f) - projection.magnitude)) * normal;


                //sliding force
                var tangent = Vector3.Cross(Vector3.up, dir);
                wallForce -= Parameters.WALL_Kappa * ((radius + 0.5f) - projection.magnitude) * Vector3.Dot(rb.velocity, tangent) * tangent;
            }
        }
       
        return wallForce*0.1f;
    }
    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (LeaderManager.IsAgent(other.gameObject) && other.gameObject==LeaderManager.leader)
        {
            isCollidingLeader = true;
            perceivedNeighbors.Add(other.gameObject);
        }
    }

    public void OnTriggerExit(Collider other)
    {
        
        if (perceivedNeighbors.Contains(other.gameObject) && other.gameObject==LeaderManager.leader)
        {
            isCollidingLeader = false;
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
