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

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
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

    private Vector3 GetProximityForce(GameObject agt)
    {
        var totalProximityForce = new Vector3();
        totalProximityForce = Vector3.zero;
        //Need to confirm?
        //Pyschological Force
        var sumRadii = radius + agt.GetComponent<NavMeshAgent>().radius;
        var com = (this.rb.centerOfMass - agt.GetComponent<Rigidbody>().centerOfMass).magnitude;
        var exp = Mathf.Exp((sumRadii - com) / Parameters.B);
        totalProximityForce += (Parameters.A * exp) * ((transform.position - agt.transform.position).normalized);

        //Penetration Force
        if (collidedNeighbors.Contains(agt))
         {
           totalProximityForce += (Parameters.k) * (sumRadii - com) * ((transform.position - agt.transform.position).normalized);
          }

        //Sliding forces to add
        //totalProximityForce+=Parameters.Kappa*(sumRadii-com)*
            
        
        return totalProximityForce;
    }

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }
    
    private Vector3 CalculateGoalForce()
    {
        var desiredDirect = (path[0] - transform.position);
        /*What should be the desired Speed?*/
        //var desiredSpeed = 10.0f;
        var desiredVel = desiredDirect.normalized * Mathf.Min(desiredDirect.magnitude, Parameters.maxSpeed);
        var calculateAcc = (desiredVel - this.GetVelocity())/Parameters.T;

        return mass*calculateAcc;
    }

    private Vector3 CalculateAgentForce()
    {
        var agentForce = new Vector3();
        agentForce = Vector3.zero;
        
        foreach (var agt in perceivedNeighbors)
        {
            if (AgentManager.IsAgent(agt))
            {
                agentForce += GetProximityForce(agt);
            }
            else
            {
                agentForce += CalculateWallForce(agt);
            }
        }
            return agentForce;
    }

    private Vector3 CalculateWallForce(GameObject agt)
    {
        var wallForce = Vector3.zero;
        var com = (rb.centerOfMass - agt.transform.position).magnitude;
        var exp = Mathf.Exp((radius - com) / Parameters.B);
        wallForce += (Parameters.A * exp) * (transform.position - agt.transform.position); //Direction?

        if (collidedNeighbors.Contains(agt))
        {
            if ((radius - com) > 0)
            {
                /*var pt=agt.GetComponent<Collision>().contacts[0];
               var dir=pt.normal;*/
                wallForce += (Parameters.k * (radius - com)) * (transform.position - agt.transform.position);
               
                //Should direction be normal to wall?

            //wallForce-=(Parameters.Kappa)*(radius-com)
            }
        }

        //Sliding force

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
