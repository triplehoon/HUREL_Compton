namespace log4netBridgeBase
{
    public class Class1
    {
        public Class1()
        {
            log4net.ILog log = log4net.LogManager.GetLogger(typeof(Class1));
            log.Info("tets");
        }
    }
}