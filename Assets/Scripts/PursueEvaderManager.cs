﻿using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class PursueEvaderManager : MonoBehaviour
{
    public int agentCount = 10;
    public float agentSpawnRadius = 20;
    public GameObject agentPrefab;
    public static Dictionary<GameObject, PursueEvader> agentsObjs = new Dictionary<GameObject, PursueEvader>();
    public Material PursuerMaterial;
    public Material EvaderMaterial;

    private static List<PursueEvader> agents = new List<PursueEvader>();
    private GameObject agentParent;
    public Vector3 destination = Vector3.forward;
    

    public const float UPDATE_RATE = 0.0f;
    private const int PATHFINDING_FRAME_SKIP = 25;

    #region Unity Functions

    void Awake()
    {
        
        Random.InitState(0);

        agentParent = GameObject.Find("Agents");
        for (int i = 0; i < agentCount; i++)
        {
            var randPos = new Vector3((Random.value - 0.5f) * agentSpawnRadius, 0, (Random.value - 0.5f) * agentSpawnRadius);
            NavMeshHit hit;
            NavMesh.SamplePosition(randPos, out hit, 10, NavMesh.AllAreas);
            randPos = hit.position + Vector3.up;

            GameObject agent = null;
            agent = Instantiate(agentPrefab, randPos, Quaternion.identity);
            agent.name = "Agent " + i;
            agent.transform.parent = agentParent.transform;
            var agentScript = agent.GetComponent<PursueEvader>();
            agentScript.radius = 0.3f;// Random.Range(0.2f, 0.6f);
            agentScript.mass = 1;
            agentScript.perceptionRadius = 20;
            if (i % 2 == 0)
            {
                agent.GetComponent<MeshRenderer>().material = EvaderMaterial;
            }
            else
            {
                agent.GetComponent<MeshRenderer>().material = PursuerMaterial;
            }

            agents.Add(agentScript);
            agentsObjs.Add(agent, agentScript);
        }

        StartCoroutine(Run());
    }

    void Update()
    {
        #region Visualization

        if (Input.GetMouseButtonDown(0))
        {
            Debug.Log("Mouse clicked");
            if (true)
            {
                var point = Camera.main.ScreenToWorldPoint(Input.mousePosition + Vector3.forward * 10);
                var dir = point - Camera.main.transform.position;
                RaycastHit rcHit;
                if (Physics.Raycast(point, dir, out rcHit))
                {
                    point = rcHit.point;

                }
            }
            else
            {
                var randPos = new Vector3((Random.value - 0.5f) * agentSpawnRadius, 0, (Random.value - 0.5f) * agentSpawnRadius);

                NavMeshHit hit;
                NavMesh.SamplePosition(randPos, out hit, 1.0f, NavMesh.AllAreas);
                print(hit.position);
                Debug.DrawLine(hit.position, hit.position + Vector3.up * 10, Color.red, 1000000);
                foreach (var agent in agents)
                {
                    //agent.ComputePath(hit.position);
                }
            }
            /*RaycastHit hitInfo = new RaycastHit();
            bool hit1 = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hitInfo);
            if (hit1)
            {

                destination = hitInfo.point;
                SetAgentDestinations(destination);

            }*/
        }
#if UNITY_EDITOR
        if (Application.isFocused)
        {
            //UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
#endif

        #endregion
    }

    IEnumerator Run()
    {
        yield return null;

        for (int iterations = 0; ; iterations++)
        {
            if (iterations % PATHFINDING_FRAME_SKIP == 0)
            {
                 //SetAgentDestinations(destination);
            }

            foreach (var agent in agents)
            {
                agent.ApplyForce();
            }

            if (UPDATE_RATE == 0)
            {
                yield return null;
            }
            else
            {
                yield return new WaitForSeconds(UPDATE_RATE);
            }
        }
    }

    #endregion

    #region Public Functions

    public static bool IsAgent(GameObject obj)
    {
        return agentsObjs.ContainsKey(obj); 
    }

    public void SetAgentDestinations(Vector3 destination)
    {
        NavMeshHit hit;
        NavMesh.SamplePosition(destination, out hit, 10, NavMesh.AllAreas);
        foreach (var agent in agents)
        {
            agent.ComputePath(hit.position);
        }
        Debug.Log("Destination is set");
    }

    public static void RemoveAgent(GameObject obj)
    {
        var agent = obj.GetComponent<PursueEvader>();

        agents.Remove(agent);
        agentsObjs.Remove(obj);
    }

    #endregion

    #region Private Functions

    #endregion

    #region Visualization Functions

    #endregion

    #region Utility Classes

    private class Tuple<K, V>
    {
        public K Item1;
        public V Item2;

        public Tuple(K k, V v)
        {
            Item1 = k;
            Item2 = v;
        }
    }

    #endregion
}
