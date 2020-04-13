using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;


public class QueueManager : MonoBehaviour
{
    public const float UPDATE_RATE = 1f;
    public GameObject Agents;

    private static Vector3 _startPos;
    private static Vector3 _servicePoint;

    private static Vector3 _nextPos;

    private static HashSet<GameObject> _agents = new HashSet<GameObject>();  
    // queue of agents - represents a physical queue in the scene
    private static Queue<Tuple<QueingAgent, Vector3>> _queue = new Queue<Tuple<QueingAgent, Vector3>>();

    // the latest entry we pop from the que and move it towards the destination
    private static Tuple<QueingAgent, Vector3> _topAgent = null; 

    
    private void Start()
    {
        _servicePoint = transform.GetChild(2).position; // position of the service_point game object
        // Debug.DrawRay(_servicePoint, Vector3.up * 10, Color.red, 100);
        _startPos = _servicePoint - new Vector3(0f, 0f, 1f);
        _nextPos = _startPos;
        
        

        // for (var i=0; i<Agents.transform.childCount; i++)
        // {
        //     Agents.transform.GetChild(i).GetComponent<QueingAgent>().ComputePath(_servicePoint);
        // }
        StartCoroutine(ProcessQueue());

    }

    private IEnumerator ProcessQueue()
    {
        
        for (var iter=0; ; iter++)
        {
            // the agent at the front of the queue, will be serviced first agent;
            if (_topAgent != null)
            {
                if (Vector3.Distance(_topAgent.Item1.transform.position, _servicePoint) < 0.7f)
                {
                    _topAgent.Item1.gameObject.SetActive(false);
                    _agents.Remove(_topAgent.Item1.gameObject);
                     
                    _topAgent = null;
                }
            }
            else
            {
                if (_queue.Count > 0)
                {
                    _topAgent = _queue.Dequeue();
                    var nextPosInQueue = _startPos;

                    _topAgent.Item1.ComputePath(_servicePoint);

                    foreach (var entry in _queue)
                    {
                        entry.Item1.ComputePath(nextPosInQueue);
                        // temp = entry.Item2;
                        // entry.Item2.Set(nextPosInQueue.x, nextPosInQueue.y, nextPosInQueue.z);
                        nextPosInQueue -= new Vector3(0, 0, 2 * _topAgent.Item1.radius + 1.5f);
                    }
                    _nextPos = nextPosInQueue;
                }
            }
            
            yield return new WaitForSeconds(UPDATE_RATE);
        }
    }
    
    private void OnTriggerEnter(Collider other)
    {
        var agentGameObject = other.gameObject;
        if (_agents.Contains(agentGameObject)) return;
        
        _agents.Add(agentGameObject);
        QueingAgent agent = agentGameObject.GetComponent<QueingAgent>();
        agent.ComputePath(_nextPos);
        // give it next available position in the queue
        _queue.Enqueue(new Tuple<QueingAgent, Vector3>(agent, _nextPos));
        // update the next available position
        _nextPos.z -= (2 * agent.radius + 1.5f);
    }
}
