using Godot;
using System;
using System.Linq.Expressions;
using System.Reflection.Emit;
using static Godot.WebSocketPeer;

public partial class node_3d : Node3D
{
	// Called when the node enters the scene tree for the first time.

	MeshInstance3D Ball;
    MeshInstance3D Anchor;
    float yA = 0.0F;
    SpringModel spring1;   //spring object
    Label3D LabelKE;  //label to display kinetic energy in gui
    Label3D LabelPE;  //label to display potential energy in gui
    Label3D LabelTotEng;
    Label3D LabelPEmgh;
    Label3D LabelY;
    Label3D LabelPESpring;
    Label3D LabelSpringDeltaL;

   Vector3 BallPos = new Vector3(0.0F, 0.0F, 0.0F);  //changed third value from yA to 0.0

    //double xA, yA, zA; // coords of anchor
    float length0 =0.9f; // natural length of pendulum
    //float length; // length of pendulum
    //double angle; // pendulum angle
    //double angleInit; // initial pendulum angle
    float time;

    //Vector3 endA; // end point of anchor
    // Vector3 endB; // end point for pendulum bob

    Vector3[] statePV = { new Vector3(-.7F, -.2F, .5F), new Vector3(0F, 0F, 0.9F) };  //initialize position and velocity vector
   

    public override void _Ready()
	{
        Anchor = GetNode<MeshInstance3D>("Anchor");
        Ball = GetNode<MeshInstance3D>("Ball");
        spring1 = GetNode<SpringModel>("SpringModel");  //stm
        spring1.GenMesh(0.2F, 0.10F, length0, 3.0F);
        LabelKE = GetNode<Label3D>("LabelKE");
        LabelPE = GetNode<Label3D>("LabelPE");
        LabelTotEng = GetNode<Label3D>("LabelTotEng");
        LabelPEmgh = GetNode<Label3D>("LabelPEmgh");
        LabelY = GetNode<Label3D>("LabelY");
        LabelPESpring = GetNode<Label3D>("LabelPESpring");
        LabelSpringDeltaL = GetNode<Label3D>("LabelSpringDeltaL");

    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{

        //yA = yA + 0.1F;  move ball up, for initial testing
        //BallPos = new Vector3(0.0F, yA, 0.0F);
        //Ball.Position = BallPos;
        //


        //Vector3 F = springK *(statePV[0])   - Fgravity;
        //Vector3[] stateVA = { statePV[1], F };  //velocity and acceleration


        statePV = RK4(statePV, time, Convert.ToSingle(delta)); //timestep 0.1 hard coded here,  
        Ball.Position = Anchor.Position + statePV[0];//
        double KE = 0.5 * BallMass * Math.Pow(statePV[1].Length(),2);
        double PE = (((BallMass * statePV[0].Y)*9.81) + 0.5 * springK * Math.Pow(((statePV[0].Length() - length0)), 2));
        double PEmgh = (BallMass * statePV[0].Y)*9.81;
        double PEspring = 0.5 * springK * Math.Pow(((statePV[0].Length() - length0)), 2);
        LabelKE.Text ="KE="+ KE.ToString();
        LabelPE.Text ="PE= "+  PE.ToString();
        LabelTotEng.Text = "Total Energy= " + (KE + PE).ToString();
        LabelPEmgh.Text = "PEmgh =" + PEmgh.ToString();
        LabelPESpring.Text = "PEspring =" + PEspring.ToString();
        LabelY.Text = "H =" + (BallMass * statePV[0].Y).ToString();
        spring1.PlaceEndPoints(Anchor.Position, Ball.Position);  //set the spring between the ball and anchor
        
    }


    //global stuff to move to top
    
    float springK = 90;  //spring constant
    static float BallMass = 1.4f; //ball mass in kg
    Vector3 Fgravity = new Vector3(0.0F, -9.81F * BallMass, 0.0F);  //vector for gravity force on ball

    public Vector3[] getVA(float time, Vector3[] state)
    {
        return new[]{ state[1], (-1.0F*springK* (state[0].Length() - length0) * state[0].Normalized() + Fgravity)/BallMass};
    }


    public Vector3[] RK4(Vector3[] statePV, float time, float delta)
    {
        // first + intermediate states
        Vector3[][] states = new Vector3[4][];
        states[0] = statePV;  //4 states, ie: the different derivatives of an RK4 timestep

        Vector3[][] stateVAs = new Vector3[4][];

        stateVAs[0] = getVA(time, statePV);

        float[] multipliers = { 0f, 0.5f, 0.5f, 1.0f };  //multipliers for k1-k4 of Rk4

        // do the intermediates
        for (int i = 1; i < 4; i++)
        {
            float step_time = time + delta * multipliers[i];  //times of intermediate steps in an RK4 step
            states[i] = new Vector3[] { 
                states[0][0] + delta * multipliers[i] * stateVAs[i - 1][0],
                states[0][1] + delta * multipliers[i] * stateVAs[i - 1][1]
            };
            stateVAs[i] = getVA(step_time,states[i]);  //calculate position for current RK step

        }

        // weighted sum of velocities/accelerations

        Vector3[] outputs = new Vector3[2];

        for (int i = 0; i < 2; i++)  //iterate through accell to velocity and velocity to position
        {
            //outputs[i] = statePV[i] + (delta / 6f) * (statePV);
           // outputs[0] = statePV[0] + (delta / 6f) * 1 * states[0][1] + 2 * states[1][1] + 2 * states[2][1] + 1 * states[3][1];
            
            outputs[i] = statePV[i] + (delta / 6f) * (1 * stateVAs[0][i] + 2 * stateVAs[1][i] + 2 * stateVAs[2][i] + 1 * stateVAs[3][i]);
        }

        return  outputs;
}



}
