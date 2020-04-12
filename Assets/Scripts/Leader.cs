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

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();


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
        bool isLeader = int.Parse(name.Split(' ')[1]) == 1;
        if (isLeader)
        {
            if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) > 1.5f)
                force += CalculateGoalForce()*0.01f;
            else
            {
                this.rb.velocity = Vector3.zero;
            }
        }
        else
        {
            if (Vector3.Distance(transform.position,leader.transform.position)>3.0f)
                force += CalculateLeaderForce(leader);
            else
            {
                rb.velocity = Vector3.zero;
                
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

        // var desiredVel = desiredVel.normalized * Mathf.Min(desiredDirect.magnitude, Parameters.maxSpeed);
        var calculateAcc = (desiredVel - this.GetVelocity()) / Parameters.T;

        return mass * calculateAcc;

    }
        private Vector3 CalculateLeaderForce(GameObject leader)
    {

        var agentForce = Vector3.zero;
        var direction = (transform.position - leader.transform.position).normalized;
        //agentForce -= (direction) * 0.5f;

        var leaderVel = leader.GetComponent<Leader>().GetVelocity();
        leaderVel = leaderVel.normalized * leaderDist;
        var aheadPos = leader.transform.position + leaderVel;
        var onWay = Vector3.Distance(aheadPos, transform.position) < 1.5f || Vector3.Distance(leader.transform.position, transform.position) < 1.5f;
        if (onWay)
        {
            Debug.Log("On the way");
            transform.position = aheadPos + Vector3.forward*1.5f;
        }
        //var direction = (transform.position - aheadPos).normalized;
        //var tangent = Vector3.Cross(Vector3.up, direction).normalized;
        agentForce -= (direction) * 0.1f;
        //agentForce += tangent * 0.01f;

        

        Debug.DrawRay(transform.position, agentForce, Color.red);
        return agentForce;
        
    }
    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (LeaderManager.IsAgent(other.gameObject) && other.gameObject!=LeaderManager.leader)
        {
            perceivedNeighbors.Add(other.gameObject);
        }
    }

    public void OnTriggerExit(Collider other)
    {
        if (perceivedNeighbors.Contains(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
    }

   
    #endregion
}
