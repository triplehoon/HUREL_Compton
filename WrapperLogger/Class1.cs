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
        public static void Debug(string className, string text)
        {
            ILog log = LogManager.GetLogger(className);
            log.Debug(text);

        }
        public static void Warn(string className, string text)
        {
            ILog log = LogManager.GetLogger(className);
            log.Warn(text);

        }

        public static void Error(string className, string text)
        {
            ILog log = LogManager.GetLogger(className);
            log.Error(text);

        }
        public static void Fatal(string className, string text)
        {
            ILog log = LogManager.GetLogger(className);
            log.Fatal(text);

        }
        
    }
}