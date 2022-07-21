using log4net;

namespace WrapperLogger
{
    public static class Log
    {
        public static void Info(string className, string text)
        {
            ILog log = LogManager.GetLogger(className);
            log.Info(text);
        } 
    }
}