using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class PursueEvader : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;

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
        force += CalculatePursueEvaderForce();
        
        if (force != Vector3.zero)
        {
            force.y = 0;
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }
    }

       private Vector3 CalculatePursueEvaderForce()
    {
        
        var agentForce = Vector3.zero;
        bool isEvader = (int.Parse(name.Split(' ')[1])%2) == 0;
        if (isEvader)
        {
            
            foreach (var agt in perceivedNeighbors)
            {

                var direction = (transform.position - agt.transform.position).normalized;
                
                var sumRadii = radius + agt.GetComponent<NavMeshAgent>().radius;
                var com = (this.rb.centerOfMass - agt.GetComponent<Rigidbody>().centerOfMass).magnitude;

                bool otherEnvader = int.Parse(agt.name.Split(' ')[1])%2 == 0;
                if (otherEnvader)
                {
                    var exp = Mathf.Exp((sumRadii - com));
                    agentForce += (exp) * (direction)*0.5f;            
                    
                }
                else
                {
                    agentForce += (direction)*0.5f;                    

                }

            }
        }
        else
        {
            foreach (var agt in perceivedNeighbors)
            {
                var direction =(transform.position - agt.transform.position).normalized;

                
                var sumRadii = radius + agt.GetComponent<NavMeshAgent>().radius;
                var com = (this.rb.centerOfMass - agt.GetComponent<Rigidbody>().centerOfMass).magnitude;

                bool envader = int.Parse(agt.name.Split(' ')[1])%2 == 0;
                if (envader)
                {
                             
                    agentForce -= (direction)*0.5f;
                                       
                }
                else
                {                    
                    var exp = Mathf.Exp((sumRadii - com));
                    agentForce += (exp) * (direction)*0.5f;
                   
                }

            }

        }
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
        if (PursueEvaderManager.IsAgent(other.gameObject))
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

   /* public void OnCollisionEnter(Collision collision)
    {
        collidedNeighbors.Add(collision.gameObject);
    }

    public void OnCollisionExit(Collision collision)
    {
        if (collidedNeighbors.Contains(collision.gameObject))
            collidedNeighbors.Remove(collision.gameObject);
    }
*/
    #endregion
}
