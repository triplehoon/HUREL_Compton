using System.Windows;
using System.Windows.Input;
using System.Windows.Controls;

using Microsoft.Xaml.Behaviors;

namespace HUREL_Imager_BEHAVIOR
{
    /// <summary>WPF 동작 연결 - 컨트롤 마우스</summary>
    public class MouseBehavior : Behavior<Control>
    {
        /// <summary>마우스 들어옴</summary>
        public static readonly DependencyProperty MouseEnterProperty =
            DependencyProperty.Register(nameof(MouseEnter), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 들어옴</summary>
        public ICommand? MouseEnter
        { get => GetValue(MouseEnterProperty) as ICommand; set => SetValue(MouseEnterProperty, value); }


        /// <summary>마우스 벗어남</summary>
        public static readonly DependencyProperty MouseLeaveProperty =
            DependencyProperty.Register(nameof(MouseLeave), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 벗어남</summary>
        public ICommand? MouseLeave
        { get => GetValue(MouseLeaveProperty) as ICommand; set => SetValue(MouseLeaveProperty, value); }


        /// <summary>마우스 버튼 눌림(터널링)</summary>
        public static readonly DependencyProperty PreviewMouseDownProperty =
            DependencyProperty.Register(nameof(PreviewMouseDown), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 버튼 눌림(터널링)</summary>
        public ICommand? PreviewMouseDown
        { get => GetValue(PreviewMouseDownProperty) as ICommand; set => SetValue(PreviewMouseDownProperty, value); }


        /// <summary>마우스 버튼 올라옴(터널링)</summary>
        public static readonly DependencyProperty PreviewMouseUpProperty =
            DependencyProperty.Register(nameof(PreviewMouseUp), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 버튼 올라옴(터널링)</summary>
        public ICommand? PreviewMouseUp
        { get => GetValue(PreviewMouseUpProperty) as ICommand; set => SetValue(PreviewMouseUpProperty, value); }


        /// <summary>마우스 움직임(터널링)</summary>
        public static readonly DependencyProperty PreviewMouseMoveProperty =
            DependencyProperty.Register(nameof(PreviewMouseMove), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 움직임(터널링)</summary>
        public ICommand? PreviewMouseMove
        { get => GetValue(PreviewMouseMoveProperty) as ICommand; set => SetValue(PreviewMouseMoveProperty, value); }


        /// <summary>마우스 버튼 두번 누름(터널링)</summary>
        public static readonly DependencyProperty PreviewMouseDoubleClickProperty =
            DependencyProperty.Register(nameof(PreviewMouseDoubleClick), typeof(ICommand), typeof(MouseBehavior), new PropertyMetadata(default));

        /// <summary>마우스 버튼 두번 누름(터널링)</summary>
        public ICommand? PreviewMouseDoubleClick
        { get => GetValue(PreviewMouseDoubleClickProperty) as ICommand; set => SetValue(PreviewMouseDoubleClickProperty, value); }

        /// <summary>연결 이벤트 처리</summary>
        protected override void OnAttached()
        {
            AssociatedObject.MouseEnter += AssociatedObject_MouseEnter;
            AssociatedObject.MouseLeave += AssociatedObject_MouseLeave;

            AssociatedObject.PreviewMouseDown += AssociatedObject_PreviewMouseDown;
            AssociatedObject.PreviewMouseUp += AssociatedObject_PreviewMouseUp;
            AssociatedObject.PreviewMouseMove += AssociatedObject_PreviewMouseMove;
            AssociatedObject.PreviewMouseDoubleClick += AssociatedObject_PreviewMouseDoubleClick;
        }

        /// <summary>연결 해제 이벤트 처리</summary>
        protected override void OnDetaching()
        {
            AssociatedObject.MouseEnter -= AssociatedObject_MouseEnter;
            AssociatedObject.MouseLeave -= AssociatedObject_MouseLeave;

            AssociatedObject.PreviewMouseDown -= AssociatedObject_PreviewMouseDown;
            AssociatedObject.PreviewMouseUp -= AssociatedObject_PreviewMouseUp;
            AssociatedObject.PreviewMouseMove -= AssociatedObject_PreviewMouseMove;
            AssociatedObject.PreviewMouseDoubleClick -= AssociatedObject_PreviewMouseDoubleClick;
        }

        /// <summary>마우스 들어옴 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_MouseEnter(object sender, MouseEventArgs e)
            => MouseEnter?.Execute(e);

        /// <summary>마우스 벗어남 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_MouseLeave(object sender, MouseEventArgs e)
            => MouseLeave?.Execute(e);

        /// <summary>마우스 버튼 눌림 터널링 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_PreviewMouseDown(object sender, MouseButtonEventArgs e)
            => PreviewMouseDown?.Execute(e);

        /// <summary>마우스 버튼 올라옴 터널링 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_PreviewMouseUp(object sender, MouseButtonEventArgs e)
            => PreviewMouseUp?.Execute(e);

        /// <summary>마우스 움직임 터널링 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_PreviewMouseMove(object sender, MouseEventArgs e)
            => PreviewMouseMove?.Execute(e);

        /// <summary>마우스 버튼 두번 누름 터널링 이벤트 처리</summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void AssociatedObject_PreviewMouseDoubleClick(object sender, MouseButtonEventArgs e)
            => PreviewMouseDoubleClick?.Execute(e);
    }
}











