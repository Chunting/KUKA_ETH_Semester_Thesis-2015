package sandbox;



import matlabcontrol.*;


public class MAT {

public static void main(String[] args) throws MatlabConnectionException, MatlabInvocationException
{
    //Create a proxy, which we will use to control MATLAB
  //  MatlabProxyFactory factory = new MatlabProxyFactory();
  //  MatlabProxy proxy = factory.getProxy();
    
    MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder().setUsePreviouslyControlledSession(true).build();

    MatlabProxyFactory factory = new MatlabProxyFactory(options);

    MatlabProxy proxy = factory.getProxy();

    proxy.disconnect();

    proxy = factory.getProxy(); // this won't open a new Matlab session
    
    
    //Create an array for this example
    //proxy.eval("fm = Focus()");
    //Invoke eval, specifying 1 argument to be returned - arguments are returned as an array
   // Object[] returnArguments = proxy.returningEval("array(2,2)", 1);
    //Retrieve the first (and only) element from the returned arguments
   //Object firstArgument = returnArguments[0];
    //Like before, cast and index to retrieve the double value
   //double innerValue = ((double[]) firstArgument)[0];
    //Print the result
   //System.out.println("Result: " + innerValue);

    //Or all in one step
   // proxy.eval("fm = Focus(1)");
   // double val = ((double[]) proxy.returningEval("fm", 1)[0])[0];
    //System.out.println("Result: " + val);
    
    
        
    //Retrieve MATLAB's release date by providing the -date argument
    //Object[] releaseDate = proxy.returningFeval("version", 1, "-date");
    //System.out.println("MATLAB Release Date: " + releaseDate[0]);
    //Display 'hello world' just like when using the demo
    
    proxy.eval("disp('hello world')");
    proxy.eval("opengl software");
    //Disconnect the proxy from MATLAB
    proxy.disconnect();
}
}